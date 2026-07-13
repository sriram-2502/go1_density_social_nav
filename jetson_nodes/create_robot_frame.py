import roslib
import rospy    
import tf
 

if __name__ == '__main__':
    rospy.init_node('camera_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((-.5, 0, 0), # this is just an esimate right now, we need to measure this to get acual numbers
                        (0, 0, 0, 1.0),
                        rospy.Time.now(),
                        "camera",
                        "robot")
        rate.sleep()
