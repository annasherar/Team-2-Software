#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"        



int main(int argc, char** argv) {

    //initialize node
    ros::init(argc, argv, "camera_node");

    //create node handler object (needed for ros with c++)
    ros::NodeHandle nh;

    //Create a publisher object 
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("camera/image", 1);

    //creates an object for converting images from OpenCV MAT formats to ROS image format
    cv_bridge::CvImage cvFrame;
    cvFrame.encoding = "bgr8";

    //Use OpenCV to capture camera feed
    cv::VideoCapture cam;
    cam.open(0); 

    if(!cam.isOpened()){
       ROS_ERROR("Failed to open camera device.");
       return -1; 
    }

    //set the desired publishing rate (Hz)
    ros::Rate loop_rate(30);  

    //keep capturing image data while ROS master is up and running or if node hasn't been told to shut down
    while(ros::ok()){

        cv:: Mat frame;
        cam.read(frame);

        //If no frame is captured, show error and break loop
        if(frame.empty()){
            ROS_ERROR("Failed to capture a frame.");
            break;
        }

        cvFrame.image = frame;
        cvFrame.header.stamp = ros::Time::now();

        //convert the cv image to a ROS image message type
        image_pub.publish(cvFrame.toImageMsg());

        cv::imshow("Webcam", frame);
        cv::waitKey(1);

        ros::spinOnce();
        loop_rate.sleep();
    }

    cam.release();
    cv::destroyAllWindows();
    
    return 0;


}