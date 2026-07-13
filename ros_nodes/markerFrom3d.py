import rospy
from autoware_msgs.msg import DetectedObjectArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion

pub = rospy.Publisher("/detections_markers", MarkerArray, queue_size=10)

def callback(msg):
    marker_array = MarkerArray()

    for i, det in enumerate(msg.objects):
        marker = Marker()
        marker.header = msg.header
        marker.header.frame_id = "camera"
        marker.ns = "detections"
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = det.pose
        marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        marker.scale.x = det.dimensions.x
        marker.scale.y = det.dimensions.y
        marker.scale.z = det.dimensions.z

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        marker_array.markers.append(marker)

    pub.publish(marker_array)

rospy.init_node("markerFrom3d")
rospy.Subscriber("/detections", DetectedObjectArray, callback)
rospy.spin()