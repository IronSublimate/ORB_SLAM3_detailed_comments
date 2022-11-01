/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>

#include "System.h"
#include "ImuTypes.h"

using namespace std;

int main(int argc, char **argv) {
//    ros::init(argc, argv, "ros901_mono_inertial");
//    ros::NodeHandle n("~");
//    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    cerr << argv[0] << "\n";
    bool bEqual = false;
    if (argc < 4 || argc > 5) {
        cerr << endl
             << "Usage: rosrun ORB_SLAM3 bag901_mono_inertial path_to_rosbag path_to_vocabulary path_to_settings [do_equalize]"
             << endl;
        return -1;
    }

    rosbag::Bag bag;
    bag.open(argv[1]);
    std::string bag_path(argv[1]);
    auto pos = bag_path.rfind('/') + 1;
    std::string bag_name = bag_path.substr(pos, bag_path.size() - pos - 4); //bag name without prefix
//    cerr<<bag_name<<"\n";

    if (argc == 5) {
        std::string sbEqual(argv[4]);
        if (sbEqual == "true")
            bEqual = true;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
//    ORB_SLAM3::System SLAM(argv[2], argv[3], ORB_SLAM3::System::IMU_MONOCULAR, true,0,{}, true);
    ORB_SLAM3::System SLAM(argv[2], argv[3], ORB_SLAM3::System::STEREO, true);

//    for(const auto& m:rosbag::View(bag)){
    rosbag::View vb(bag);
//    std::ofstream of("imu2.csv");
    cv::Mat left, right;
    for (auto it = vb.begin(); it != vb.end(); ++it) {
        if (it->getTopic() == "/miivii_gmsl_ros_raw_front_node/camera0/compressed") {
            auto img_ptr = it->instantiate<sensor_msgs::CompressedImage>();
            auto cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::MONO8);
//            cv::Mat image=cv_ptr->image({0,0,cv_ptr->image.cols,640});
            cv_ptr->image({0, 640, cv_ptr->image.cols, cv_ptr->image.rows - 640}) = 111;
            left = std::move(cv_ptr->image);
            if (not right.empty()) {
                SLAM.TrackStereo(left, right, cv_ptr->header.stamp.toSec());
                left = cv::Mat();
                right = cv::Mat();
            }
//            SLAM.TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
//            std::this_thread::sleep_for(100ms);
        } else if (it->getTopic() == "/miivii_gmsl_ros_raw_front_node/camera1/compressed") {
            auto img_ptr = it->instantiate<sensor_msgs::CompressedImage>();
            auto cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::MONO8);
//            cv::Mat image=cv_ptr->image({0,0,cv_ptr->image.cols,640});
            cv_ptr->image({0, 640, cv_ptr->image.cols, cv_ptr->image.rows - 640}) = 111;
            right = std::move(cv_ptr->image);
            if (not left.empty()) {
                SLAM.TrackStereo(left, right, cv_ptr->header.stamp.toSec());
                left = cv::Mat();
                right = cv::Mat();
            }
        } else {

        }
//        break;
    }
    SLAM.Shutdown();
    string save_path = "./output/" + bag_name + ".txt";
    SLAM.SaveTrajectoryTUM(save_path);
    bag.close();
    return 0;
}


