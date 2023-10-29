import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2

 
def camera_node():

    #Initialize Node
    rospy.init_node('camera_node', anonymous = True)

    #Create an image publisher object
    image_pub = rospy.Publisher('camera_image', Image, queue_size = 10)

    #Use open cv to get camera footage
    cam = cv2.VideoCapture(0)

    bridge = CvBridge()

    #Set the publishing rate (here its 10hz)
    rate = rospy.rate(10)

    while not rospy.is_shutdown():
        ret, frame = cam.read()

        if(ret):
            image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            image_msg.header = Header()
            image_msg.header.stamp = rospy.Time.now()
            image_pub.publish(image_msg)
        
        rate.sleep()


if __name__ == '__main__':
    try:
        camera_node()
    except rospy.ROSInterruptException:
        pass
        
