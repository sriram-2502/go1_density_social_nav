import rospy
from vision_msgs.msg import Detection3DArray, Detection3D
import pyzed.sl as sl
import cv2
import numpy as np


def get_pos():
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_units = sl.UNIT.METER
    init_params.camera_resolution = sl.RESOLUTION.VGA
    init_params.camera_fps = 15

    zed.open(init_params)
    
    
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

    zed.enable_positional_tracking(tracking_parameters)

    camera_pose = sl.Pose()

    while not rospy.is_shutdown():
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.get_position(camera_pose)

            py_translation = sl.Translation()
            tx = round(camera_pose.get_translation(py_translation).get()[0], 3)
            ty = round(camera_pose.get_translation(py_translation).get()[1], 3)
            tz = round(camera_pose.get_translation(py_translation).get()[2], 3)
            print("Translation: tx: {0}, ty:  {1}, tz:  {2}, timestamp: {3}\n".format(tx, ty, tz, camera_pose.timestamp))

            #Display orientation quaternion
            py_orientation = sl.Orientation()
            ox = round(camera_pose.get_orientation(py_orientation).get()[0], 3)
            oy = round(camera_pose.get_orientation(py_orientation).get()[1], 3)
            oz = round(camera_pose.get_orientation(py_orientation).get()[2], 3)
            ow = round(camera_pose.get_orientation(py_orientation).get()[3], 3)
            print("Orientation: ox: {0}, oy:  {1}, oz: {2}, ow: {3}\n".format(ox, oy, oz, ow))



        #pub.publish(msg)
        #rate.sleep()

    zed.disable_positional_tracking()
    zed.close()

if __name__ == "__main__":
    try:
        get_pos()
    except rospy.ROSInterruptException:
        pass