import roslib
import rospy    
import tf
 

if __name__ == '__main__':
    rospy.init_node('camera_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((-.12, 0, .12),
                        (0, 0, 0, 1.0),
                        rospy.Time.now(),
                        "robot",
                        "camera")
        rate.sleep()
