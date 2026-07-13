import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from autoware_msgs.msg import DetectedObject, DetectedObjectArray
import pyzed.sl as sl
import cv2
from cv_bridge import CvBridge
import numpy as np
import tf
import rosbag
from datetime import datetime

def camera_node():
    """
    initialize zed camera
    """
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_units = sl.UNIT.METER
    init_params.camera_resolution = sl.RESOLUTION.VGA
    init_params.camera_fps = 15
    init_params.sdk_verbose = 1
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD

    # Open the camera
    err = zed.open(init_params)
    if err > sl.ERROR_CODE.SUCCESS:
        print("Camera Open : "+repr(err)+". Exit program.")
        exit()

    # start object detection
    obj_param = sl.ObjectDetectionParameters()
    obj_param.enable_tracking=True
    obj_param.enable_segmentation=True
    obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_MEDIUM

    if obj_param.enable_tracking :
        positional_tracking_param = sl.PositionalTrackingParameters()
        #positional_tracking_param.set_as_static = True
        zed.enable_positional_tracking(positional_tracking_param)

    print("Object Detection: Loading Module...")

    err = zed.enable_object_detection(obj_param)
    if err != sl.ERROR_CODE.SUCCESS :
        print("Enable object detection : "+repr(err)+". Exit program.")
        zed.close()
        exit()

    # start positional tracking
    tracking_parameters = sl.PositionalTrackingParameters()
    tracking_parameters.mode = sl.POSITIONAL_TRACKING_MODE.GEN_1
    tracking_parameters.depth_min_range             = .2
    tracking_parameters.enable_area_memory          = False
    tracking_parameters.enable_pose_smoothing       = False 
    tracking_parameters.set_floor_as_origin         = True
    tracking_parameters.enable_imu_fusion           = True 
    tracking_parameters.set_as_static               = False
    tracking_parameters.set_gravity_as_origin       = True 
    tracking_parameters.enable_localization_only    = False # Localize only, do not update the map
    tracking_parameters.enable_2d_ground_mode       = True # Enable 2D ground-constrained tracking

    err = zed.enable_positional_tracking(tracking_parameters)
    if err != sl.ERROR_CODE.SUCCESS :
        print("Enable object detection : "+repr(err)+". Exit program.")
        zed.close()
        exit()


    """
    setup zed outputs
    """ 
    # pose output
    camera_pose = sl.Pose()

    # image output
    frame_zed = sl.Mat()
    bridge = CvBridge()

    # Detection Output
    objects = sl.Objects()
    # Detection runtime parameters
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 40
    zed.set_object_detection_runtime_parameters(obj_runtime_param) # can be set at any time


    """
    setup ros topics
    """
    # detections topic
    rospy.init_node('camera_node', anonymous=True)
    pub_det = rospy.Publisher("detections", DetectedObjectArray, queue_size=10) 
    
    
    # pose topic
    pub_pos = rospy.Publisher("camera_pose", PoseStamped, queue_size=10) 
    position = PoseStamped()
    
    # image topic
    pub_img = rospy.Publisher("video", Image, queue_size=10)
    frame = Image()

    # camera frame
    br = tf.TransformBroadcaster()
    
    rate = rospy.Rate(10)  # 10hz

    """
    record data
    """

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    videoBag    = rosbag.Bag(f'bags/video_{timestamp}.bag', 'w')
    posBag      = rosbag.Bag(f'bags/pos{timestamp}.bag', 'w')


    """
    main loop
    """
    while not rospy.is_shutdown():
        #retrive data
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_objects(objects)
            zed.retrieve_image(frame_zed, sl.VIEW.LEFT)
            zed.get_position(camera_pose)
            obj_array = objects.object_list

        # format data to ros msg
        detections = DetectedObjectArray()
        for obj in obj_array:
            det = DetectedObject()

            corners = np.array(obj.bounding_box)
            center = np.mean(corners, axis=0)

            min_vals = np.min(corners, axis=0)
            max_vals = np.max(corners, axis=0)
            size = max_vals - min_vals
            
            det.pose.position.x = center[0]
            det.pose.position.y = center[1]
            det.pose.position.z = center[2]

            det.dimensions.x = size[0]
            det.dimensions.y = size[1]
            det.dimensions.z = size[2]
            
            det.velocity.linear.x   = obj.velocity[0]
            det.velocity.linear.y   = obj.velocity[1]
            det.velocity.linear.z   = obj.velocity[2]

            det.id                  = obj.id

            det.header.stamp    = rospy.Time.now()
            det.header.frame_id = "camera"

            detections.objects.append(det)
        
        py_translation = sl.Translation()
        position.pose.position.x = round(camera_pose.get_translation(py_translation).get()[0], 3)
        position.pose.position.y = round(camera_pose.get_translation(py_translation).get()[1], 3)
        position.pose.position.z = round(camera_pose.get_translation(py_translation).get()[2], 3)
        
        
        #Display orientation quaternion
        py_orientation = sl.Orientation()
        position.pose.orientation.x = round(camera_pose.get_orientation(py_orientation).get()[0], 3)
        position.pose.orientation.y = round(camera_pose.get_orientation(py_orientation).get()[1], 3)
        position.pose.orientation.z = round(camera_pose.get_orientation(py_orientation).get()[2], 3)
        position.pose.orientation.w = round(camera_pose.get_orientation(py_orientation).get()[3], 3)
        
        frame_bgr = cv2.cvtColor(frame_zed.get_data(), cv2.COLOR_RGBA2BGR)
        frame = bridge.cv2_to_imgmsg(frame_bgr, encoding="rgb8")

        # set msg headers
        #detections.header = Header()
        detections.header.stamp     = rospy.Time.now()
        detections.header.frame_id  = "camera"
        position.header.stamp       = rospy.Time.now()
        position.header.frame_id    = "world"
        frame.header.stamp          = rospy.Time.now()
        
        # publish and bag
        pub_det.publish(detections)
        pub_pos.publish(position)
        posBag.write("position", position, position.header.stamp)
        pub_img.publish(frame)
        videoBag.write("video", frame, frame.header.stamp)
        rate.sleep()

        
        # broadcast camera frame 
        br.sendTransform((position.pose.position.x,position.pose.position.y,position.pose.position.z),
                        (position.pose.orientation.x,position.pose.orientation.y,position.pose.orientation.z,position.pose.orientation.w),
                        rospy.Time.now(),
                        "camera",
                        "world")

    videoBag.close()
    posBag.close()
    
    # Close the camera
    zed.disable_positional_tracking()
    zed.disable_object_detection()
    zed.close()
    

if __name__ == "__main__":
    try:
        camera_node()
    except rospy.ROSInterruptException:
        pass