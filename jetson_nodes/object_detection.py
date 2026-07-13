import rospy
from vision_msgs.msg import Detection3DArray, Detection3D
import pyzed.sl as sl
import cv2
import numpy as np

def object_detection():
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_units = sl.UNIT.METER
    init_params.sdk_verbose = 1

    # Open the camera
    err = zed.open(init_params)
    if err > sl.ERROR_CODE.SUCCESS:
        print("Camera Open : "+repr(err)+". Exit program.")
        exit()

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

    # Detection Output
    objects = sl.Objects()
    # Detection runtime parameters
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 40
    zed.set_object_detection_runtime_parameters(obj_runtime_param) # can be set at any time

    rospy.init_node('object_detection', anonymous=True)
    pub = rospy.Publisher("detections", Detection3DArray, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        zed.grab()
        zed.retrieve_objects(objects)
        obj_array = objects.object_list

        msg = Detection3DArray()
        msg.header.stamp = rospy.Time.now()

        for obj in obj_array:
            det = Detection3D()

            corners = np.array(obj.bounding_box)
            center = np.mean(corners, axis=0)

            min_vals = np.min(corners, axis=0)
            max_vals = np.max(corners, axis=0)
            size = max_vals - min_vals
            
            det.bbox.center.position.x = center[2]
            det.bbox.center.position.y = -center[0]
            det.bbox.center.position.z = -center[1]

            det.bbox.size.x = size[2]
            det.bbox.size.y = size[0]
            det.bbox.size.z = size[1]

            print(f"Object at {det.bbox.center.position.x:.2f}, {det.bbox.center.position.y:.2f}, {det.bbox.center.position.z:.2f}")

            msg.detections.append(det)

        pub.publish(msg)
        rate.sleep()

    # Close the camera
    zed.disable_object_detection()
    zed.close()

if __name__ == "__main__":
    try:
        object_detection()
    except rospy.ROSInterruptException:
        pass