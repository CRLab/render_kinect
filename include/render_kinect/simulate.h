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
/* Header file that sets up the simulator and triggers the simulation 
 * of the kinect measurements and stores the results under a given directory.
 */
#ifndef SIMULATE_H
#define SIMULATE_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <string.h>

#include <render_kinect/kinectSimulator.h>
#include <render_kinect/cnpy.h>

namespace render_kinect {

	class Simulate {
	public:

		Simulate(CameraInfo &cam_info, const std::string &object_name, const std::string &dot_path)
		{
			// allocate memory for depth image
			int w = cam_info.width;
			int h = cam_info.height;

			depth_im_ = cv::Mat(h, w, CV_32FC1);
			scaled_im_ = cv::Mat(h, w, CV_32FC1);

			object_model_ = new KinectSimulator(cam_info, object_name, dot_path);

			transform_ = Eigen::Affine3d::Identity();

		}

		~Simulate() {
			delete object_model_;
		}

		void simulateMeasurement(const Eigen::Affine3d &new_tf) {
			// empty previous measurements
			cloud_.clear();

			// update old transform
			transform_ = new_tf;

			// simulate measurement of object and store in image, point cloud and labeled image
			object_model_->intersect(transform_, point_cloud_, depth_im_, labels_);

			// in case object is not in view, don't store any data
			// However, if background is used, there will be points in the point cloud
			// although they don't belong to the arm
			int n_vis = 4000;
			if(point_cloud_.rows < n_vis) {
				std::cout << "Object not in view.\n";
				return;
			}

			// Fill in the cloud data
			cloud_.width = static_cast<uint32_t>(point_cloud_.rows);
			cloud_.height = 1;
			cloud_.is_dense = false;
			cloud_.points.resize(cloud_.width * cloud_.height);

			for (int i = 0; i < point_cloud_.rows; i++) {
				const float* point = point_cloud_.ptr<float>(i);
				cloud_.points[i].x = point[0];
				cloud_.points[i].y = point[1];
				cloud_.points[i].z = point[2];
			}
		}

		void save_pcd(const std::string &out_file_path) {
			if(cloud_.empty()) {
				std::cout << "Point cloud is empty. Nothing to save" << std::endl;
				return;
			}

			assert(boost::algorithm::ends_with(outFilePath, ".pcd"));

			if (pcl::io::savePCDFileBinary(out_file_path, cloud_) != 0)
				std::cout << "Couldn't store point cloud at " << out_file_path << std::endl;
		}

		void save_labels(const std::string &out_file_path) {
            assert(boost::algorithm::ends_with(out_file_path, ".png"));
            cv::imwrite(out_file_path, labels_);
		}

        void save_depth(const std::string &out_file_path) {
            assert(boost::algorithm::ends_with(out_file_path, ".png"));
            convertScaleAbs(depth_im_, scaled_im_, 255.0f);
            cv::imwrite(out_file_path, scaled_im_);
		}

		void save_transform(const std::string &out_file_path) {
		    assert(boost::algorithm::ends_with(out_file_path, ".npy"));
		    double* matrix = transform_.matrix().data();
		    cnpy::npy_save(out_file_path, matrix, {4, 4}, "w");
		}

		KinectSimulator *object_model_;
		cv::Mat depth_im_, scaled_im_, point_cloud_, labels_;
		pcl::PointCloud<pcl::PointXYZ> cloud_;
		Eigen::Affine3d transform_;

	};

} //namespace render_kinect
#endif // SIMULATE_H
