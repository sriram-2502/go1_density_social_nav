import rospy
from vision_msgs.msg import Detection3DArray, Detection3D
import numpy as np

# global variable to store latest detections
latest_detections = []

def detection_callback(msg):
    global latest_detections
    latest_detections = msg.detections
    rospy.loginfo(f"Received {len(latest_detections)} detections")

def object_subscriber():
    rospy.init_node('object_subscriber')

    rospy.Subscriber("/detections", Detection3DArray, detection_callback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        # use stored detections later
        for det in latest_detections:
            pos = det.bbox.center.position

            x = pos.x
            y = pos.y
            z = pos.z

            print(f"Object at {x:.2f}, {y:.2f}, {z:.2f}")

        rate.sleep()

if __name__ == "__main__":
    object_subscriber()