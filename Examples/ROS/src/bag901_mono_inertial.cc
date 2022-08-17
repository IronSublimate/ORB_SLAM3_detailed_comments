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

class ImuGrabber {
public:
    ImuGrabber() {};

    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber {
public:
    ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb, const bool bClahe) : mpSLAM(pSLAM), mpImuGb(pImuGb),
                                                                                    mbClahe(bClahe) {}

    void GrabImage(const sensor_msgs::CompressedImageConstPtr &msg);

    cv::Mat GetImage(const sensor_msgs::CompressedImageConstPtr &img_msg);

    void SyncWithImu();

    queue<sensor_msgs::CompressedImageConstPtr> img0Buf;
    std::mutex mBufMutex;

    ORB_SLAM3::System *mpSLAM;
    ImuGrabber *mpImuGb;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};


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
    ImuGrabber imugb;
    ImageGrabber igb(&SLAM, &imugb, bEqual); // TODO

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

//    // Maximum delay, 5 seconds
//    ros::Subscriber sub_imu = n.subscribe("/ULV_imu_std_msg", 1000, &ImuGrabber::GrabImu, &imugb);
//    ros::Subscriber sub_img0 = n.subscribe("/miivii_gmsl_ros_raw_front_node/camera0/compressed", 100,
//                                           &ImageGrabber::GrabImage, &igb);
//
//    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);
//
//    ros::spin();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::CompressedImageConstPtr &img_msg) {
    mBufMutex.lock();
    if (!img0Buf.empty())
        img0Buf.pop();
    img0Buf.push(img_msg);
    mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::CompressedImageConstPtr &img_msg) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0) {
        return cv_ptr->image.clone();
    } else {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void ImageGrabber::SyncWithImu() {
    while (1) {
        cv::Mat im;
        double tIm = 0;
        if (!img0Buf.empty() && !mpImuGb->imuBuf.empty()) {
            tIm = img0Buf.front()->header.stamp.toSec();
            if (tIm > mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;
            {
                this->mBufMutex.lock();
                im = GetImage(img0Buf.front());
                img0Buf.pop();
                this->mBufMutex.unlock();
            }

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mpImuGb->mBufMutex.lock();
            if (mpImuGb->imuBuf.size() > 1) {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (mpImuGb->imuBuf.size() > 1 && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm) {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x,
                                    mpImuGb->imuBuf.front()->linear_acceleration.y,
                                    mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x,
                                    mpImuGb->imuBuf.front()->angular_velocity.y,
                                    mpImuGb->imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();
            if (mbClahe)
                mClahe->apply(im, im);

            mpSLAM->TrackMonocular(im, tIm, vImuMeas);
        }

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg) {
    mBufMutex.lock();
    if (not imuBuf.empty()) {
        auto last = imuBuf.back().get();
        if (imu_msg->header.stamp.toSec() - last->header.stamp.toSec() < 0.002) {
            ros::Duration d(0.005);
//            auto t = imu_msg->header.stamp;
            const_cast<sensor_msgs::Imu *>(last)->header.stamp = imu_msg->header.stamp - d;
//            const_cast<std::remove_const_t<decltype(last)>>(last)->header.stamp = imu_msg->header.stamp;
//            last->header.stamp.sec = t.sec;
//            last->header.stamp.sec = t.sec;
        }
        std::cerr << (last)->header.stamp - imu_msg->header.stamp << "\n";
    }
    imuBuf.push(imu_msg);
    mBufMutex.unlock();

    return;
}


