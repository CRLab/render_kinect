/*********************************************************************
 *
 *  Copyright (c) 2014, Jeannette Bohg - MPI for Intelligent System
 *  (jbohg@tuebingen.mpg.de)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Jeannette Bohg nor the names of MPI
 *     may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* This is a C++ reimplementation of the kinect sensor model as proposed 
   in 
   
   @incollection{Gschwandtner11b,
   title = {{BlenSor: Blender Sensor Simulation Toolbox Advances in Visual Computing}},
   author = {Gschwandtner, Michael and Kwitt, Roland and Uhl, Andreas and Pree, Wolfgang},
   pages = {199--208},
   publisher = {Springer Berlin / Heidelberg},
   series = {Lecture Notes in Computer Science},
   year = {2011}
   }

   and implemented as a python plugin for blender.
}

*/

#include <render_kinect/kinectSimulator.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <render_kinect/camera.h>
#include <render_kinect/gaussian.h>
#include <render_kinect/perlin.h>
#include <render_kinect/simplex.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/AABB_polyhedron_triangle_primitive.h>

// for activating the asserts
// #undef NDEBUG

#include <assert.h>

#ifdef HAVE_OPENMP
#include <omp.h>
#endif

// DEBUG
// static unsigned countf = 0;
// static const int prec = 5;

namespace render_kinect {

  float getRGB( uint8_t r, uint8_t g, uint8_t b){
    union{ int intp; float floatp; } a;
    int res = (int(r) << 16) | (int(g) << 8) | int(b);
    a.intp=res;
    float rgb = *(&a.floatp);
    return rgb;
  };

  struct color_cmp {
    
    bool operator()(cv::Scalar a, cv::Scalar b)
    {
      float mag_a = std::sqrt(a(0)*a(0) + a(1)*a(1) + a(2)*a(2));
      float mag_b = std::sqrt(b(0)*b(0) + b(1)*b(1) + b(2)*b(2));
      return mag_a < mag_b;
    }
    
  };

  // Constructor
  KinectSimulator::KinectSimulator(const CameraInfo &p_camera_info,
				   std::vector<std::string> &object_paths,
				   std::string dot_path,
				   bool background,
				   std::string room_path,
				   const Eigen::Affine3d &room_tf) 
    : render_bg_(background)
    , camera_( p_camera_info)
    , noise_type_(p_camera_info.noise_)
    , noise_gen_(NULL)
    , noisy_labels_(0)
    , room_tf_(room_tf)
  {
    
    if(render_bg_)
      std::cout << "Background rendering with " << room_path << std::endl;

    std::cout << "Width and Height: " << p_camera_info.width_ << "x"
	      << p_camera_info.height_ << std::endl;
    std::cout << "Loading models for "<<  object_paths.size() << " objects " <<  std::endl;
    
    // Add all object models
    std::set<cv::Scalar, color_cmp> color_map_set;
    std::vector<std::string>::const_iterator it;
    for(it = object_paths.begin(); it!=object_paths.end(); ++it){
      boost::shared_ptr<ObjectMeshModel> model(new ObjectMeshModel(*it));
      bool unique = false;
      std::pair<std::set<cv::Scalar>::iterator, bool> ret;
      models_.push_back(model);
      while(!unique){
	ret = color_map_set.insert(cv::Scalar( rand()&255, rand()&255, rand()&255 ));
	unique = ret.second;
      }
    }

    std::cout << "Size of color_map " << color_map_set.size() << std::endl;
    std::copy(color_map_set.begin(), color_map_set.end(), std::back_inserter(color_map_));
    

    // add background mesh and color label
    if(render_bg_){
      room_ = boost::shared_ptr<ObjectMeshModel>(new ObjectMeshModel(room_path));
      color_map_.push_back(cv::Scalar(background_, background_, background_));
    }

    search_ = new TreeAndTri; 
    updateTree();
    
    // reading dot pattern for filtering of disparity image
    dot_pattern_ = cv::imread(dot_path.c_str(), 0);
    if(! dot_pattern_.data ) // Check for invalid input
      {
	std::cout <<  "Could not load dot pattern from " <<  dot_path << std::endl ;
	exit(-1);
      }
    
    // initialize filter matrices for simulated disparity
    weights_ = cv::Mat(size_filt_,size_filt_,CV_32FC1);
    for(int x=0; x<size_filt_; ++x){
      float *weights_i = weights_.ptr<float>(x);
      for(int y=0; y<size_filt_; ++y){
	int tmp_x = x-size_filt_/2;
	int tmp_y = y-size_filt_/2;
	if(tmp_x!=0 && tmp_y!=0)
	  weights_i[y] = 1.0/(sq(1.2*(float)tmp_x)+sq(1.2*(float)tmp_y));
	else 
	  weights_i[y] = 1.0;
      }
    }

    fill_weights_ = cv::Mat(size_filt_, size_filt_, CV_32FC1);
    for(int x=0; x<size_filt_; ++x){
      float *weights_i = fill_weights_.ptr<float>(x);
      for(int y=0; y<size_filt_; ++y){
	int tmp_x = x-size_filt_/2;
	int tmp_y = y-size_filt_/2;
	if(std::sqrt(sq(tmp_x)+sq(tmp_y)) < 3.1)
	  weights_i[y] = 1.0/(1.0+sq(tmp_x)+sq(tmp_y));
	else 
	  weights_i[y] = -1.0;
      }
    }

    // Extracting noise type and setting up noise generator
    if(noise_type_==GAUSSIAN)
      {
	//Gaussian Noise
	float mean = 0.0;
	float std  = 0.15;
	noise_gen_ = new GaussianNoise( camera_.getWidth(), camera_.getHeight(), mean, std);
      } else if (noise_type_==PERLIN) 
      {
	float scale = 0.4;
	noise_gen_ = new PerlinNoise( camera_.getWidth(), camera_.getHeight(), scale);
      } else if (noise_type_==SIMPLEX) 
      {
	float scale = 0.5;
	noise_gen_ = new SimplexNoise( camera_.getWidth(), camera_.getHeight(), scale);
      }

  }

  // Destructor
  KinectSimulator::~KinectSimulator() 
  {
    if(noise_gen_!=NULL)
      delete noise_gen_;
  }

  void KinectSimulator::setOriginalTransform(const std::vector<Eigen::Affine3d> &part_mesh_transforms)
  {
    std::vector<boost::shared_ptr<ObjectMeshModel> >::const_iterator it;
    for(it=models_.begin();it!=models_.end();++it)
      {
	(*it)->setOriginalTransform(part_mesh_transforms[it-models_.begin()]);
      }
  }

  void KinectSimulator::setRoomTransform(Eigen::Affine3d &room_tf){room_tf_ = room_tf;}

  sensor_msgs::CameraInfoPtr KinectSimulator::getCameraInfo (ros::Time time)
  {
    return camera_.getCameraInfo(time);
  }
  
  // Function that exchanges current object transform
  void KinectSimulator::updateObjectPoses(const std::vector<Eigen::Affine3d> &p_transforms)   
  {
    //std::cout << "Size of Models: " << models_.size() << std::endl;
    //std::cout << "Size of transforms " << p_transforms.size() << std::endl;

    assert(p_transforms.size()==models_.size());
    
    std::vector<boost::shared_ptr<ObjectMeshModel> >::const_iterator it;
    for(it=models_.begin();it!=models_.end();++it)
      (*it)->updateTransformation(p_transforms[it-models_.begin()]);

    if(render_bg_)
      room_->updateTransformation(room_tf_);
  }
  
  // Function that triggers AABB tree update by exchanging old vertices 
  // with new vertices according to the updated transform
  void KinectSimulator::updateTree()
  {
    int last_v = 0, last_t = 0;
    std::vector<boost::shared_ptr<ObjectMeshModel> >::const_iterator it;
    for(it=models_.begin();it!=models_.end();++it)
      {
	int nv = (*it)->uploadVertices(search_, last_v);
	int nt = (*it)->uploadIndices(search_, last_t, last_v);
	(*it)->uploadPartIDs(search_, last_t, it - models_.begin());
	last_v = nv;
	last_t = nt;
      }

    if(render_bg_) {
      room_->uploadVertices(search_, last_v);
      room_->uploadIndices(search_, last_t, last_v);
      room_->uploadPartIDs(search_, last_t, color_map_.size()-1);
      
    }

    search_->tree.rebuild(search_->triangles.begin(), search_->triangles.end());
    search_->tree.accelerate_distance_queries();
  }

  // Function that intersects rays with the object model at current state.
  void KinectSimulator::intersect(const std::vector<Eigen::Affine3d> &p_transforms, 
				  cv::Mat &point_cloud,
				  cv::Mat &rgb_vals,
				  cv::Mat &depth_map,
				  cv::Mat &labels) 
  {
    // However, for articulated or multiple rigid objects that change their configuration over 
    // time, this update is necessary and is therefore kept in this code.
    // If only  one object would be tracked, you could also change the camera pose. An update of the 
    // AABB tree would not be needed.
    updateObjectPoses(p_transforms);
    updateTree();

    // allocate memory for depth map and labels
    depth_map = cv::Mat(camera_.getHeight(), camera_.getWidth(), CV_64FC1);
    depth_map.setTo(0.0);
    labels = cv::Mat(camera_.getHeight(), camera_.getWidth(), CV_8UC3);
    labels.setTo(cv::Scalar(background_, background_, background_));
    cv::Mat disp(camera_.getHeight(), camera_.getWidth(), CV_32FC1);
    disp.setTo(invalid_disp_);
    // go through the whole image and create a ray from a pixel -> dir    
    std::vector<cv::Point3f> vec;
    std::vector<float> rgbs;
    int n_occluded = 0;
    vec.reserve(camera_.getHeight() * camera_.getWidth());
    rgbs.reserve(camera_.getHeight() * camera_.getWidth());

#if HAVE_OMP
#pragma omp parallel for collapse(2)
#endif
    for(int c=0; c<camera_.getWidth(); ++c) {
      for(int r=0; r<camera_.getHeight(); ++r) {
	// compute ray from pixel and camera configuration
	cv::Point3f ray = camera_.projectPixelTo3dRay(cv::Point2f(c,r));
	// check if there is any intersection of the ray with an object by do_intersect
	uint32_t reach_mesh = search_->tree.do_intersect(Ray(Point(0,0,0), Vector(ray.x, ray.y, ray.z)));
	if (reach_mesh){
	  // if there is one or many intersections, order them according to distance to camera 
	  // and continue computation with closest
	  std::list<Object_and_Primitive_id> intersections;
	  search_->tree.all_intersections(Ray(Point(0,0,0),Vector( ray.x, ray.y, ray.z)), 
					  std::back_inserter(intersections));
	  if(!intersections.empty()) {
	    std::list<Object_and_Primitive_id>::const_iterator it;
	    Point min_p(0,0,0);
	    double min_dist = std::numeric_limits<double>::infinity();
	    double min_id = -1;
	    for (it = intersections.begin(); it != intersections.end(); ++it) {
	      CGAL::Object object = it->first;
	      std::size_t triangle_id = std::distance(search_->triangles.begin(),it->second); 
	      assert( search_->triangles[triangle_id] == *(it->second) ); 
	      Point point;
	      if(CGAL::assign(point,object)){
		double dist = abs(point);
		if(dist<min_dist){
		  // distance to point
		  min_dist = dist;
		  // intersection coordinates
		  min_p = point;
		  // label of the intersected object
		  min_id = triangle_id;
		}
	      } else {
		std::cout << "Intersection object is NOT a point ?????" << std::endl;
	      }
	    }

	    // check if point is also visible in second camera by casting a ray to this point
	    uint32_t reach_mesh_r = search_->tree.do_intersect(Ray(Point(camera_.getTx(),0,0), 
								   Point(min_p.x(), min_p.y(), min_p.z())));
	    if(reach_mesh_r) {
	      // if there are intersections, get the closest to the camera
	      std::list<Object_and_Primitive_id> intersections_r;
	      search_->tree.all_intersections(Ray(Point(camera_.getTx(),0,0), Point(min_p.x(), min_p.y(), min_p.z())), 
					      std::back_inserter(intersections_r));
	      if(!intersections_r.empty()) {
		std::list<Object_and_Primitive_id>::const_iterator id;
		Point min_p_r(min_p.x(), min_p.y(), min_p.z());
		double min_dist_r = std::numeric_limits<double>::infinity();
		for (id = intersections_r.begin(); id != intersections_r.end(); ++id) {
		  CGAL::Object object = id->first;
		  Point point;
		  if(CGAL::assign(point,object)){
		    double dist = abs(point);
		    if(dist<min_dist_r){
		      min_dist_r = dist;
		      min_p_r = point;
		    }
		  }
		}
		
		// check if closest intersection is the same as from the left image
		// if not, point is not visible in both images -> occlusion boundary
		Point diff(min_p_r.x() - min_p.x(), min_p_r.y() - min_p.y(), min_p_r.z() - min_p.z());
		if(abs(diff)<0.0001) {
		  // get pixel position of ray in right image
		  float tx_fx = camera_.getTx();
		  cv::Point3d point_right( min_p.x()-tx_fx, min_p.y(), min_p.z());
		  cv::Point2f right_pixel = camera_.project3dToPixel(point_right);
		  // quantize right_pixel
		  right_pixel.x = round(right_pixel.x*8.0)/8.0;
		  right_pixel.y = round(right_pixel.y*8.0)/8.0;
		  // compute disparity image
		  float quant_disp = c - right_pixel.x; 
		  float* disp_i = disp.ptr<float>(r);
		  disp_i[(int)c] = quant_disp;
		  // fill label image with part id 
		  unsigned char* labels_i = labels.ptr<unsigned char>(r);
		  cv::Scalar color = color_map_[search_->part_ids[min_id]];
		  for(int col=0; col<3; ++col){
		    labels_i[(int)c*3+col] = color(col);
		  }
		} else {
		  n_occluded++;
		}
	      } // if there are non-zero intersections from right camera
	    } // if mesh reached from right camera
	  } // if non-zero intersections
	} // if mesh reached 
      } // camera_.getHeight()
    } // camera_.getWidth()


    // Filter disparity image and add noise 
    cv::Mat out_disp, out_labels;
    filterDisp(disp, labels, out_disp, out_labels);
    if(noisy_labels_)
      labels = out_labels;

    //Go over disparity image and recompute depth map and point cloud after filtering and adding noise etc
    for(int r=0; r<camera_.getHeight(); ++r) {
      float* disp_i = out_disp.ptr<float>(r);
      double* depth_map_i = depth_map.ptr<double>(r);
      unsigned char* labels_i = labels.ptr<unsigned char>(r);
      for(int c=0; c<camera_.getWidth(); ++c) {
	float disp = disp_i[c];
	if(disp<invalid_disp_){
	  cv::Point3d new_p;
	  new_p.z = (camera_.getFx()*camera_.getTx())/disp;
	  if(new_p.z<camera_.getZNear() || new_p.z>camera_.getZFar()){
	    continue;
	  }
	  new_p.x = (new_p.z/ camera_.getFx()) * (c - camera_.getCx());
	  new_p.y = (new_p.z/ camera_.getFy()) * (r - camera_.getCy());
	  vec.push_back(new_p);
	  depth_map_i[(int)c] = new_p.z;
	  
	  // store float-encoded rgb
	  unsigned char r = labels_i[(int)c*3+0];
	  unsigned char g = labels_i[(int)c*3+1];
	  unsigned char b = labels_i[(int)c*3+2];
	  rgbs.push_back(getRGB(r,g,b));
	}
      }
    }
    point_cloud = cv::Mat(vec).reshape(1).clone();
    rgb_vals = cv::Mat(rgbs).clone();
  }

  // filter disparity with a 9x9 correlation window
  void KinectSimulator::filterDisp(const cv::Mat& disp, const cv::Mat& labels, cv::Mat& out_disp, cv::Mat& out_labels)
  {
    cv::Mat interpolation_map = cv::Mat::zeros(disp.rows,disp.cols, CV_32FC1);
    
    cv::Mat noise_field;
    if(noise_type_==NONE)
      noise_field = cv::Mat::zeros(disp.rows,disp.cols, CV_32FC1);
    else 
      // generate noise field according to given noise type
      // can be Gaussian, Perlin or Simplex
      noise_gen_->generateNoiseField(noise_field);

    //DEBUG
    //countf++;
    //std::stringstream lS;
    //lS << "/tmp/noise"<< std::setw(prec) << std::setfill('0') << countf << ".png";
    //cv::imwrite(lS.str().c_str(), (noise_field+1)*128);

    // mysterious parameter
    float noise_smooth = 1.5;

    // initialise output arrays
    out_disp = cv::Mat(disp.rows, disp.cols, disp.type());
    out_disp.setTo(invalid_disp_);
    if(noisy_labels_){
      out_labels = cv::Mat(labels.rows, labels.cols, labels.type());
      out_labels.setTo(background_);
    }

    // determine filter boundaries
    unsigned lim_rows = std::min(camera_.getHeight()-size_filt_, dot_pattern_.rows-size_filt_);
    unsigned lim_cols = std::min(camera_.getWidth()-size_filt_, dot_pattern_.cols-size_filt_);
    int center = size_filt_/2.0;
    for(unsigned r=0; r<lim_rows; ++r) {
      const float* disp_i = disp.ptr<float>(r+center);
      const unsigned char* labels_i;
      if(noisy_labels_)
	labels_i = labels.ptr<unsigned char>(r+center);
      const float* dots_i = dot_pattern_.ptr<float>(r+center);
      
      float* out_disp_i = out_disp.ptr<float>(r+center);
      unsigned char* out_labels_i;
      if(noisy_labels_)
	out_labels_i = out_labels.ptr<unsigned char>(r+center);
      
      float* noise_i =  noise_field.ptr<float>((int)((r+center)/noise_smooth));
      
      // window shifting over disparity image
      for(unsigned c=0; c<lim_cols; ++c) {
	if( dots_i[c + center]>0 && disp_i[c + center] < invalid_disp_){
	  cv::Rect roi = cv::Rect(c, r, size_filt_, size_filt_);
	  cv::Mat window = disp(roi);
	  cv::Mat dot_win = dot_pattern_(roi);
	  // check if we are at a occlusion boundary without valid disparity values
	  // return value not binary but between 0 or 255
	  cv::Mat valid_vals = (window<invalid_disp_);
	  cv::Mat valid_dots;
	  cv::bitwise_and( valid_vals, dot_win, valid_dots);
	  cv::Scalar n_valids = cv::sum(valid_dots)/255.0;
	  cv::Scalar n_thresh = cv::sum(dot_win)/255.0;
	  
	  // only add depth value at center of window if there are more 
	  // valid disparity values than 2/3 of the number of dots
	  if( n_valids(0) > n_thresh(0)/1.5 ) {
	    // compute mean only over the valid values of disparities in that window
	    cv::Scalar mean = cv::mean(window, valid_vals);
	    // weighted deviation from mean
	    cv::Mat diffs = cv::abs(window-mean);
	    cv::multiply( diffs, weights_, diffs);
	    // get valid values that fall on dot pattern
	    cv::Mat valids = (diffs<window_inlier_distance_);
	    cv::bitwise_and( valids, valid_dots, valid_dots);
	    n_valids = cv::sum(valid_dots)/255.0;

	    // only add depth value at center of window if there are more 
	    // valid disparity values than 2/3 of the number of dots
	    if(n_valids(0)>n_thresh(0)/1.5) {
	      float accu = window.at<float>(center,center);
	      assert(accu<invalid_disp_);
	      out_disp_i[c + center] = round((accu + noise_i[(int)((c+center)/noise_smooth)])*8.0)/8.0;
	      if(noisy_labels_) {
		if(out_disp_i[c + center]<invalid_disp_)
		  for(int col=0; col<3; ++col)
		    out_labels_i[(c + center)*3+col] = labels_i[(c+center)*3+col];
	      }
	      
	      cv::Mat interpolation_window = interpolation_map(roi);
	      cv::Mat disp_data_window = out_disp(roi);
	      cv::Mat label_data_window;
	      if(noisy_labels_)
		label_data_window = out_labels(roi);
	      cv::Mat substitutes = interpolation_window < fill_weights_;
	      fill_weights_.copyTo( interpolation_window, substitutes);
	      disp_data_window.setTo(out_disp_i[c + center], substitutes);
	      
	      if(noisy_labels_) {
		for(int sr=0;sr<substitutes.rows; ++sr){
		  unsigned char* subs_i = substitutes.ptr<unsigned char>(sr);
		  unsigned char* label_win_i = label_data_window.ptr<unsigned char>(sr);
		  for(int sc=0;sc<substitutes.cols; ++sc){
		    if(subs_i[sc]>0 && out_disp_i[c + center] < invalid_disp_){
		      for(int col=0; col<3; ++col)
			label_win_i[sc*3+col] = out_labels_i[(c + center)*3+col]; 
		    }
		  }
		}
	      }
	    }
	  } 
	}
      }
    }
  }

}    //namespace render_kinect
