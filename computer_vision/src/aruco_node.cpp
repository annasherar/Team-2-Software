#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>     
#include  <image_transport/image_transport.h>
#include "std_msgs/Int8.h"
#include <opencv2/opencv.hpp>
#include <iostream>
              
void arucoCallback(const sensor_msgs::ImageConstPtr& rosFrame){

    std::cout << "started arucoCallback" << std::endl;

    try{

       
        //Convert ROS image message to OpenCV format
        //cv_bridge::CvImagePtr cvFrame = cv_bridge::toCvCopy(rosFrame, "bgr8");
        cv_bridge::CvImagePtr cvFrame;
        cvFrame = cv_bridge::toCvCopy(rosFrame, sensor_msgs::image_encodings::BGR8);
        cv:: Mat frame = cvFrame->image;
        std::cout <<"Started openCV" << std::endl;
        cv::imshow("webcam", frame);
        cv::waitKey(1); 

              

        //std::cout <<frame<< std::endl;
       
    
    }

    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    

}

int main(int argc, char** argv) {

   //Initialize Node and nodehandler
    ros::init(argc, argv, "aruco_node");
    ros::NodeHandle nh;

    
    //Create Image Transport object and create subscriber object using ImageTransport class
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imageSub = it.subscribe("camera/image",1,arucoCallback);

    //Subscribe using the ros general publisher
    //ros::Subscriber imageSub = nh.subscribe<sensor_msgs::Image>("camera/image", 1, arucoCallback);

    std::cout << "Aruco Node initialized" << std::endl;

    //Create a ROS publisher for if an aruco marker is detected
    //ros::Publisher detected_pub = nh.advertise<std_msgs::Bool>("arucoDetected", 10);
    //std_msgs::Bool arucoDetected;


    //Create a ROS publisher for the aruco marker ID
    //ros::Publisher Id_pub = nh.advertise<std_msgs::Int8>("arucoID", 10);
    //std_msgs::Int8 arucoID;
   
   cv::namedWindow("webcam");

    while(ros::ok()){
        std::cout << "Before Spin" << std::endl;
        ros::spin();
    }

    cv::destroyAllWindows();
    

    return 0;
   


}