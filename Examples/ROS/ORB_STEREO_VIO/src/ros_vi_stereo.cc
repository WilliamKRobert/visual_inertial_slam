/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

#include "MsgSync/MsgSynchronizer.h"

#include "../../../src/IMU/imudata.h"
#include "../../../src/IMU/configparam.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

using namespace std;

class SensorGrabber
{
public:
    SensorGrabber(ORB_SLAM2::System* pSLAM, 
                  const double imageMsgDelaySec, 
                  const double testDiscardTime,
                  const double g3dm,
                  bool bAccMultiply98,
                  bool do_rectify=true)
    :mpSLAM(pSLAM),     
    _imageMsgDelaySec(imageMsgDelaySec),
    _testDiscardTime(testDiscardTime),
    _g3dm(g3dm),
    _bAccMultiply98(bAccMultiply98), 
    _do_rectify(do_rectify),
    bFirstRun(true){}

    void GrabImuStereo(const std::vector<sensor_msgs::ImuConstPtr> msgImu, 
                       const sensor_msgs::ImageConstPtr& msgLeft, 
                       const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;

    double _imageMsgDelaySec;
    double _testDiscardTime;
    const double _g3dm;

    bool _bAccMultiply98;
    bool _do_rectify;
    
    bool bFirstRun;
    double initTime;

    cv::Mat M1l, M2l, M1r, M2r;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo Visual-Inertial Localization and Mapping");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ORB_SLAM2::ConfigParam config(argv[2]);

    /**
     * @brief added data sync
     */
    double imageMsgDelaySec = config.GetImageDelayToIMU();
    ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);
    ros::NodeHandle nh;
    ros::Subscriber leftimagesub;
    ros::Subscriber rightimagesub;
    ros::Subscriber imusub;
    if(ORB_SLAM2::ConfigParam::GetRealTimeFlag())
    {
        leftimagesub = nh.subscribe(config._leftImageTopic, /*200*/ 2, &ORBVIO::MsgSynchronizer::imageCallback, &msgsync);
        rightimagesub = nh.subscribe(config._rightImageTopic, /*200*/ 2, &ORBVIO::MsgSynchronizer::imageCallback, &msgsync);
        imusub = nh.subscribe(config._imuTopic, 200, &ORBVIO::MsgSynchronizer::imuCallback, &msgsync);
    }
    sensor_msgs::ImageConstPtr leftImageMsg;
    sensor_msgs::ImageConstPtr rightImageMsg;
    std::vector<sensor_msgs::ImuConstPtr> vimuMsg;

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
    const bool bAccMultiply98 = config.GetAccMultiply9p8();

    ros::Rate r(1000);
    bool do_rectify = true;
    SensorGrabber igb(&SLAM, imageMsgDelaySec, config._testDiscardTime, g3dm, bAccMultiply98, do_rectify);
    if(!ORB_SLAM2::ConfigParam::GetRealTimeFlag())
    {    
        ROS_WARN("Get sensor data from ros bag file");

        std::string bagfile = config._bagfile;
        rosbag::Bag bag;
        bag.open(bagfile,rosbag::bagmode::Read);

        std::vector<std::string> topics;
        std::string imutopic = config._imuTopic;
        std::string leftimagetopic = config._leftImageTopic;
        std::string rightImagetopic = config._rightImageTopic;
        topics.push_back(leftimagetopic);
        topics.push_back(rightimagetopic);
        topics.push_back(imutopic);

        rosbag::View view(bag, rosbag::TopicQuery(topics));


        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::ImuConstPtr simu = m.instantiate<sensor_msgs::Imu>();
            if(simu!=NULL)
                msgsync.imuCallback(simu);
            sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();
            if(simage!=NULL)
                msgsync.imageCallback(simage);
            bool bdata = msgsync.getRecentMsgs(leftImageMsg, rightImageMsg, vimuMsg);

            if(bdata)
            {
                igb.GrabImuStereo(vimuMsg, leftImageMsg, rightImageMsg);

                // Wait local mapping end.
                bool bstop = false;
                while(!SLAM.bLocalMapAcceptKF())
                {
                    if(!ros::ok())
                    {
                        bstop=true; 
                    }
                };
                if(bstop)
                    break;

            }

            //cv::waitKey(1);

            ros::spinOnce();
            r.sleep();
            if(!ros::ok())
                break;
        }

    }
//     else
//     {
//         ROS_WARN("Run realtime");
//         while(ros::ok())
//         {
//             bool bdata = msgsync.getRecentMsgs(leftImageMsg,rightImageMsg, vimuMsg);

//             if(bdata)
//             {
                
//                 igb.GrabImuStereo(vimuMsg, leftImageMsg, rightImageMsg);
//             }

//             //cv::waitKey(1);

//             ros::spinOnce();
//             r.sleep();
//             if(!ros::ok())
//                 break;
//         }
//     }

// //    ImageGrabber igb(&SLAM);

// //    ros::NodeHandle nodeHandler;
// //    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

// //    ros::spin();


//     // Save camera trajectory
//     //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
//     SLAM.SaveKeyFrameTrajectoryNavState(config._tmpFilePath+"KeyFrameNavStateTrajectory.txt");

//     cout<<endl<<endl<<"press any key to shutdown"<<endl;
//     getchar();

//     // Stop all threads
//     SLAM.Shutdown();

//     ros::shutdown();

    return 0;
}

void SensorGrabber::GrabImuStereo(const std::vector<sensor_msgs::ImuConstPtr> msgImu, 
                                  const sensor_msgs::ImageConstPtr& msgLeft, 
                                  const sensor_msgs::ImageConstPtr& msgRight){
    std::vector<ORB_SLAM2::IMUData> vimuData;
    //ROS_INFO("image time: %.3f",imageMsg->header.stamp.toSec());
    for(unsigned int i=0;i<msgImu.size();i++)
    {
        sensor_msgs::ImuConstPtr imuMsg = msgImu[i];
        double ax = imuMsg->linear_acceleration.x;
        double ay = imuMsg->linear_acceleration.y;
        double az = imuMsg->linear_acceleration.z;
        if(_bAccMultiply98)
        {
            ax *= _g3dm;
            ay *= _g3dm;
            az *= _g3dm;
        }
        ORB_SLAM2::IMUData imudata(imuMsg->angular_velocity.x,
                                   imuMsg->angular_velocity.y,
                                   imuMsg->angular_velocity.z,
                                   ax, ay, az,
                                   imuMsg->header.stamp.toSec());
        vimuData.push_back(imudata);
        //ROS_INFO("imu time: %.3f",msgImu[i]->header.stamp.toSec());
    }


    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (bFirstRun){
        bFirstRun = false;
        initTime = msgLeft->header.stamp.toSec();
    }

    cv::Mat imLeft, imRight;
    if(_do_rectify)
    {
        cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
        
    }
    else
    {   
        imLeft = cv_ptrLeft->image;
        imRight = cv_ptrRight->image;

    }

    if (msgLeft->header.stamp.toSec() < initTime + _testDiscardTime){
            imLeft = cv::Mat::zeros(imLeft.rows, imLeft.cols, imLeft.type());
            imRight = cv::Mat::zeros(imRight.rows, imRight.cols, imRight.type());
    }

    mpSLAM->TrackStereoVI(vimuData,
                          imLeft,
                          imRight,
                          cv_ptrLeft->header.stamp.toSec() - _imageMsgDelaySec);

}



