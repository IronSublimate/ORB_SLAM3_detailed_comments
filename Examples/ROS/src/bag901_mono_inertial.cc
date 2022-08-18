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

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include "System.h"
#include "ImuTypes.h"

using namespace std;

int main(int argc, char **argv) {
//    ros::init(argc, argv, "ros901_mono_inertial");
//    ros::NodeHandle n("~");
//    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    bool bEqual = false;
    if (argc < 4 || argc > 5) {
        cerr << endl
             << "Usage: rosrun ORB_SLAM3 901_mono_inertial path_to_rosbag path_to_vocabulary path_to_settings [do_equalize]"
             << endl;
        return -1;
    }

    rosbag::Bag bag;
    bag.open(argv[1]);

    if (argc == 5) {
        std::string sbEqual(argv[4]);
        if (sbEqual == "true")
            bEqual = true;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[2], argv[3], ORB_SLAM3::System::IMU_MONOCULAR, true);
//
//    for(const auto& m:rosbag::View(bag)){
    rosbag::View vb(bag);
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    for (auto it = vb.begin(); it != vb.end(); ++it) {
        if (it->getTopic() == "/ULV_imu_std_msg") {
//            cerr<<m.getTime()<<"\n";
            auto imu_ptr = it->instantiate<sensor_msgs::Imu>();
//            imu_ptr->header.stamp = it->getTime();
            vImuMeas.emplace_back(
                    imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y, imu_ptr->linear_acceleration.z,
                    imu_ptr->angular_velocity.x, imu_ptr->angular_velocity.y, imu_ptr->angular_velocity.z,
                    it->getTime().toSec()
            );
        } else if (it->getTopic() == "/miivii_gmsl_ros_raw_front_node/camera0/compressed") {
            auto img_ptr = it->instantiate<sensor_msgs::CompressedImage>();
            auto cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::MONO8);

            SLAM.TrackMonocular(cv_ptr->image, it->getTime().toSec(), vImuMeas);
            vImuMeas.clear();
        } else {

        }
    }

    return 0;
}


