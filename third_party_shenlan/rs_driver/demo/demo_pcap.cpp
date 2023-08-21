/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#include <rs_driver/api/lidar_driver.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <rs_driver/msg/point_cloud_msg.hpp>
#endif

typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

using namespace robosense::lidar;

//
// @brief point cloud callback function. The caller should register it to the lidar driver.
//        Via this fucntion, the driver fetches an EMPTY point cloud message from the caller.
// @param msg  The empty point cloud message.
//
std::shared_ptr<PointCloudMsg> pointCloudGetCallback(void)
{
  // Note: This callback function runs in the packet-parsing thread of the driver, 
  //       so please DO NOT do time-consuming task here.
  return std::make_shared<PointCloudMsg>();
}

//
// @brief point cloud callback function. The caller should register it to the lidar driver.
//        Via this function, the driver returns a STUFFED point cloud message to the caller. 
// @param msg  The stuffed point cloud message.
//
void pointCloudPutCallback(std::shared_ptr<PointCloudMsg> msg)
{
  // Note: This callback function runs in the packet-parsing thread of the driver, 
  //       so please DO NOT do time-consuming task here. Instead, process it in another thread.
  RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;
}

//
// @brief exception callback function. The caller should register it to the lidar driver.
// @param code The error code to represent the error/warning/information
//
void exceptionCallback(const Error& code)
{
  // Note: This callback function runs in the packet-receving/packet-parsing thread of the driver, 
  //       so please DO NOT do time-consuming task here.
  RS_WARNING << code.toString() << RS_REND;
}

int main(int argc, char* argv[])
{
  RS_TITLE << "------------------------------------------------------" << RS_REND;
  RS_TITLE << "            RS_Driver Core Version: v" << getDriverVersion() << RS_REND;
  RS_TITLE << "------------------------------------------------------" << RS_REND;

  RSDriverParam param;                                         ///< Create a parameter object
  param.input_type = InputType::PCAP_FILE;
  param.input_param.pcap_path = "/home/robosense/lidar.pcap";  ///< Set the pcap file directory
  param.input_param.msop_port = 6699;                          ///< Set the lidar msop port number, the default is 6699
  param.input_param.difop_port = 7788;                         ///< Set the lidar difop port number, the default is 7788
  param.lidar_type = LidarType::RSM1;                          ///< Set the lidar type. Make sure this type is correct

  param.print();

  LidarDriver<PointCloudMsg> driver;  ///< Declare the driver object
  driver.regPointCloudCallback(pointCloudGetCallback, pointCloudPutCallback); ///< Register the point cloud callback function 
  driver.regExceptionCallback(exceptionCallback);  ///< Register the exception callback function
  if (!driver.init(param))                         ///< Call the init function
  {
    RS_ERROR << "Driver Initialize Error..." << RS_REND;
    return -1;
  }
  driver.start();  ///< The driver thread will start
  RS_DEBUG << "RoboSense Lidar-Driver Linux pcap demo start......" << RS_REND;

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}
