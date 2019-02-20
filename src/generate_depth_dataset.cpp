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

/* main_kinect.cpp
 * Test program that sets up simulator with specific camera parameters
 * and object mesh. A number of object poses is sampled from which
 * a desired measured output (depthmap, label image, point cloud) is
 * generated and stored.
 */

#include <boost/program_options.hpp>

#include <render_kinect/simulate.h>
#include <render_kinect/camera.h>

#include <iostream>
#include <fstream>

namespace po = boost::program_options;

/* Sampling of random 6DoF transformations. */
void getRandomTransform(const double &p_x,
                        const double &p_y,
                        const double &p_z,
                        const double &p_angle,
                        Eigen::Affine3d &p_tf) {
    Eigen::Vector3d axis(((double) (rand() % 1000)) / 1000.0,
                         ((double) (rand() % 1000)) / 1000.0,
                         ((double) (rand() % 1000)) / 1000.0);
    Eigen::Vector3d t(p_x * (double) (rand() % 2000 - 1000) / 1000,
                      p_y * (double) (rand() % 2000 - 1000) / 1000,
                      p_z * (double) (rand() % 2000 - 1000) / 1000);
    p_tf = Eigen::Affine3d::Identity();
    p_tf.translate(t);
    p_tf.rotate(Eigen::AngleAxisd(p_angle * (double) (rand() % 2000 - 1000) / 1000, axis));
}

void parse_args(int argc, char** argv, std::string &mesh_filepath, std::string &kinect_pattern_path) {
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "produce help message")
            ("mesh_filepath", po::value<std::string>(), "Object model to be rendered")
            ("kinect_pattern_path", po::value<std::string>()->default_value("../data/kinect-pattern_3x3.png"), "Path to kinect pattern file")
            ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        exit(1);
    }

    if (vm.count("mesh_filepath")) {
        mesh_filepath = vm["mesh_filepath"].as<std::string>();
    } else {
        std::cout << "mesh_filepath not set" <<std::endl;
        std::cout << desc << std::endl;
        exit(-1);
    }

    kinect_pattern_path = vm["kinect_pattern_path"].as<std::string>();
}

// main function that generated a number of sample outputs for a given object mesh.
int main(int argc, char **argv) {

    // Path to the object mesh model.
    std::string mesh_filepath;
    // Path to the dot pattern
    std::string kinect_pattern_path;
    parse_args(argc, argv, mesh_filepath, kinect_pattern_path);

    // Camera Parameters
    render_kinect::CameraInfo cam_info{};

    cam_info.width = 640;
    cam_info.height = 480;
    cam_info.cx_ = 320;
    cam_info.cy_ = 240;

    cam_info.z_near = 0.5;
    cam_info.z_far = 6.0;
    cam_info.fx_ = 580.0;
    cam_info.fy_ = 580.0;
    // baseline between IR projector and IR camera
    cam_info.tx_ = 0.075;

    // Type of noise
    //  cam_info.noise_ = render_kinect::GAUSSIAN;
    //  cam_info.noise_ = render_kinect::PERLIN;
    cam_info.noise_ = render_kinect::NONE;

    // Test Transform
    Eigen::Affine3d transform(Eigen::Affine3d::Identity());
    transform.translate(Eigen::Vector3d(0.089837, -0.137769, 0.949210));
    transform.rotate(Eigen::Quaterniond(0.906614, -0.282680, -0.074009, -0.304411));

    // Kinect Simulator
    render_kinect::Simulate Simulator(cam_info, mesh_filepath, kinect_pattern_path);

    // Number of samples
    int frames = 10;

    // Storage of random transform
    Eigen::Affine3d noise;
    for (int i = 0; i < frames; ++i) {

        // sample noisy transformation around initial one
        getRandomTransform(0.02, 0.02, 0.02, 0.05, noise);
        Eigen::Affine3d current_tf = noise * transform;

        // give pose and object name to renderer
        Simulator.simulateMeasurement(current_tf);

        std::stringstream pcd_path;
        pcd_path << "/tmp/pointcloud_" << i << ".pcd";
        Simulator.save_pcd(pcd_path.str());

        std::stringstream depth_path;
        depth_path << "/tmp/depth_" << i << ".png";
        Simulator.save_depth(depth_path.str());

        std::stringstream labels_path;
        labels_path << "/tmp/labels_" << i << ".png";
        Simulator.save_labels(labels_path.str());

        std::stringstream transform_path;
        transform_path << "/tmp/model_rotation_" << i << ".npy";
        Simulator.save_transform(transform_path.str());
    }

    return 0;
}
