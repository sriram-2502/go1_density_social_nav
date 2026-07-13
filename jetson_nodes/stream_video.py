import rospy
from sensor_msgs.msg import Image
import pyzed.sl as sl
import cv2
from cv_bridge import CvBridge
import numpy as np

def stream_video():
    #Create a ZED camera object
    zed = sl.Camera()

    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.VGA
    init_params.camera_fps = 15

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(-1)
    
    rospy.init_node('stream_video', anonymous=True)
    pub = rospy.Publisher("video", Image, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    frame_zed = sl.Mat()
    bridge = CvBridge()
    msg = Image()

    while not rospy.is_shutdown():
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # A new image is available if grab() returns SUCCESS
            zed.retrieve_image(frame_zed, sl.VIEW.LEFT) # Retrieve the left image
            
        frame = cv2.cvtColor(frame_zed.get_data(), cv2.COLOR_RGBA2BGR)
        msg = bridge.cv2_to_imgmsg(frame, encoding="rgb8")
        msg.header.stamp = rospy.Time.now()

        pub.publish(msg)
        rate.sleep()

    # Close the camera
    zed.disable_object_detection()
    zed.close()

if __name__ == "__main__":
    try:
        stream_video()
    except rospy.ROSInterruptException:
        pass