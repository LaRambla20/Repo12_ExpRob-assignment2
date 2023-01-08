/**
 * \file marker_cli.cpp
 * \author Emanuele Rambaldi
 * \date December 2022
 * \version 0.1
 * \brief Client node used for the acquisition of information from aruco markers.
 * \details
 *
 * Subscribes to: <br>
 *  /robot/camera1/image_raw
 *
 * Publishes to: <br>
 *  /robot/camera1/marker_info
 *
 * Services : <br>
 *  /room_info
 *
 * Description:
 * This node implements first of all a subscriber. It subscribes to the topic on which the camera of the robot publishes the acquired images.
 * This information is processed so as to eventually retrieve the ID of markers contained in the visual field of the camera. Then, the implemented client comes into play.
 * A request message containing the ID is then sent to the 'marker_server', which returns information about the corresponding location (if any). 
 * This response message is used to fill in another message that is published by a publisher on a topic, to which the 'state_machine' node is subscribed.
 */

#include "ros/ros.h"
#include <cstdlib>
#include <assignment2/RoomConnection.h> //custom msg
#include <assignment2/RoomFeatures.h> //custom msg
#include <assignment2/RoomInformation.h> //custom srv
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

/** 
* \class ArucoMarkerClient
* \brief Class for the acquisition of information from aruco markers.
*
* This class contains the call-back function that is invoked every time that a camera acquisition is provided. 
*/

class ArucoMarkerClient
{
private:
  // ArUco stuff
  aruco::MarkerDetector mDetector_;
  std::vector<aruco::Marker> markers_;
  aruco::CameraParameters camParam_;
  assignment2::RoomInformation srv;

  // node params
  double marker_size_;
  bool useCamInfo_;

  // ROS pub-sub
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;  /**< subsriber to the /robot/camera1/image_raw topic */

  // ROS pub
  ros::Publisher marker_info_pub_; /**< publisher onto the /robot/camera1/marker_info topic */

  // ROS cli
  ros::ServiceClient image_client_; /**< client belonging to the /room_info topic */

  cv::Mat inImage_;
  
public:
  /** 
  * \brief Constructor that initialises the subscriber, the publisher and the client.
  */
  ArucoMarkerClient() :
      nh_("~"), it_(nh_), useCamInfo_(true)
  {
    image_sub_ = it_.subscribe("/robot/camera1/image_raw", 1, &ArucoMarkerClient::image_callback, this);
    // ROS pub (to forward the marker info to the 'state_machine' node)
    marker_info_pub_ = nh_.advertise<assignment2::RoomFeatures>("/robot/camera1/marker_info", 100);
    // ROS cli (to retrieve info about the detected markers form the server)
    image_client_ = nh_.serviceClient<assignment2::RoomInformation>("/room_info");
    
    nh_.param<bool>("use_camera_info", useCamInfo_, false);
    camParam_ = aruco::CameraParameters();
  }

  /**
  * \brief Function that is called every time that a new message is published on the '/robot/camera1/image_raw' topic.
  * \param msg variable containing the published message passed by reference
  *
  * This function processes the incoming message so as to eventually retrieve the ID of markers contained in the visual field of the camera.
  * A request message containing the ID is then sent to the 'marker_server', which returns information about the corresponding location (if any). 
  * This response message is used to fill in another message that is published by a publisher on a topic, to which the 'state_machine' node is subscribed.
  */

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    // bool that is True if the number of subscribers to the 'marker_info' topic is > 0
    bool publishMarkerInfo = marker_info_pub_.getNumSubscribers() > 0;

    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      inImage_ = cv_ptr->image;
   
      // clear out previous detection results
      markers_.clear();

      // ok, let's detect
      mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);

        for (std::size_t i = 0; i < markers_.size(); ++i)
        {
            std::cout << "The id of the detected marker is: ";
            std::cout << markers_.at(i).id << " ";
            std::cout << std::endl;

            // call the server with the retrieved marker ID
            srv.request.id = markers_.at(i).id; // retreive the ID of the i-th marker detected
            if (image_client_.call(srv)) // call the server with the retrieved ID
                {   
                    // print the response
                    ROS_INFO("Room information:\n Room: %s;\n Coord: (%f,%f);\n", srv.response.room.c_str(), srv.response.x, srv.response.y);
                    for (std::size_t j = 0; j < srv.response.connections.size(); ++j){
                        ROS_INFO("Connected to room: %s Through door: %s;\n", srv.response.connections.at(j).connected_to.c_str(), srv.response.connections.at(j).through_door.c_str());
                    }
                    
                    // if the number of subscribers to the 'marker_info' topic is > 0
                    if (publishMarkerInfo)
                    {
                      assignment2::RoomFeatures out_msg;
                      out_msg.room = srv.response.room;
                      out_msg.x = srv.response.x;
                      out_msg.y = srv.response.y;
                      // for each connection, store the info in the corresponding slot of the published message
                      for (std::size_t j = 0; j < srv.response.connections.size(); ++j){
                        out_msg.connections.push_back(srv.response.connections.at(j));
                      }

                      // publish the response on the 'marker_info' topic
                      marker_info_pub_.publish(out_msg);
                    }
                }
            else
                {
                    ROS_ERROR("Failed to call service");
                }
        }

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
};


/**
* \brief Main function.
*
* This function simply initialises the node, instantiates an instance of the class and spins to allow the cyclical execution of this mechanism.
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "room_info_client"); // initialise the node

  ArucoMarkerClient node; // instantiate an instance of the class

  ros::spin();
}