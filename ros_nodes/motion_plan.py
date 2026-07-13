import rospy
from vision_msgs.msg import Detection3DArray, Detection3D
from unitree_legged_msgs.msg import HighCmd
import numpy as np

# global variable to store latest detections
latest_detections = []

HIGHLEVEL = 0  # common for unitree_ros_to_real; verify via rostopic echo /high_cmd

def detection_callback(msg):
    global latest_detections
    latest_detections = msg.detections
    rospy.loginfo(f"Received {len(latest_detections)} detections")

def set_u8_array(msg, field_name, values, length=None):
    """
    Assign uint8[] fields in rospy safely (often represented as immutable bytes).
    values: iterable of ints [0..255]
    """
    if not hasattr(msg, field_name):
        return
    if length is not None:
        values = list(values)[:length] + [0] * max(0, length - len(list(values)))
        values = values[:length]
    setattr(msg, field_name, bytes([int(v) & 0xFF for v in values]))

def set_u32_array(msg, field_name, values, length=None):
    """Assign uint32[] fields (these are usually mutable lists, but assign whole to be safe)."""
    if not hasattr(msg, field_name):
        return
    vals = list(int(v) for v in values)
    if length is not None:
        vals = vals[:length] + [0] * max(0, length - len(vals))
        vals = vals[:length]
    setattr(msg, field_name, vals)

def fill_defaults(cmd: HighCmd):
    # Header + level
    set_u8_array(cmd, "head", [0xFE, 0xEF], length=2)
    cmd.levelFlag = int(HIGHLEVEL) & 0xFF
    cmd.frameReserve = 0

    # IDs (safe to zero)
    set_u32_array(cmd, "SN", [0, 0], length=2)
    set_u32_array(cmd, "version", [0, 0], length=2)
    cmd.bandWidth = 0

    # Control fields
    cmd.mode = 0
    cmd.gaitType = 0
    cmd.speedLevel = 0

    cmd.footRaiseHeight = 0.0
    cmd.bodyHeight = 0.0

    # float arrays are mutable lists in rospy, usually safe:
    cmd.position[0] = 0.0
    cmd.position[1] = 0.0

    cmd.euler[0] = 0.0
    cmd.euler[1] = 0.0
    cmd.euler[2] = 0.0

    cmd.velocity[0] = 0.0
    cmd.velocity[1] = 0.0
    cmd.yawSpeed = 0.0

    # bms
    cmd.bms.off = 0
    cmd.bms.reserve = bytes([0, 0, 0])

def motion_plan():
    rospy.init_node('motion_plan')
    rospy.Subscriber("/detections", Detection3DArray, detection_callback)

    topic = rospy.get_param("~topic", "/high_cmd")
    hz = float(rospy.get_param("~hz", 10.0))

    pub = rospy.Publisher(topic, HighCmd, queue_size=10)
    rate = rospy.Rate(hz)

    stand_time = float(2.0)
    walk_time  = float(5.0)

    gait = int(1)
    speed = int(1)
    foot_h = float(0.08)
    body_h = float(0.0)

    t0 = rospy.Time.now()
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        if (rospy.Time.now() - t0).to_sec() > 2.0:
            rospy.logwarn("No subscribers on %s yet; continuing anyway.", topic)
            break
        rospy.sleep(0.05)

    stand = HighCmd()
    fill_defaults(stand)
    stand.mode = 1
    stand.gaitType = gait
    stand.speedLevel = speed
    stand.footRaiseHeight = foot_h
    stand.bodyHeight = body_h

    rospy.loginfo("FORCE_STAND %.1fs @ %.1fHz", stand_time, hz)
    end_t = rospy.Time.now() + rospy.Duration.from_sec(stand_time)
    while rospy.Time.now() < end_t and not rospy.is_shutdown():
        pub.publish(stand)
        rate.sleep()

    # Phase 2: WALK
    walk = HighCmd()
    fill_defaults(walk)
    walk.mode = 2
    walk.gaitType = gait
    walk.speedLevel = speed
    walk.footRaiseHeight = foot_h
    walk.bodyHeight = body_h
    
    # rospy.loginfo("WALK %.1fs: vx=%.2f vy=%.2f wz=%.2f", walk_time, vx, vy, wz)
    end_t = rospy.Time.now() + rospy.Duration.from_sec(walk_time)
    while rospy.Time.now() < end_t and not rospy.is_shutdown():

        # use stored detections later
        for det in latest_detections:
            x = det.bbox.center.position.x
            y = det.bbox.center.position.y
            z = det.bbox.center.position.z

            bboxx = det.bbox.size.x
            bboxy = det.bbox.size.y

            rx = np.sqrt(bboxx**2 + bboxy*bboxx) #radius of elipse enclosing object in the x direction
            ry = np.sqrt(bboxy**2 + bboxx*bboxy) #radius of elipse enclosing object in the y direction

            print(f"Object at {x:.2f}, {y:.2f}, {z:.2f}")
            print(f"Object size rx={rx:.2f}, ry={ry:.2f}")

        walk.velocity[0] = 0.20
        walk.velocity[1] = 0
        walk.yawSpeed = 0

        pub.publish(walk)
        rate.sleep()
    
    
    # Phase 3: STOP (walk mode, zero velocities)
    stop = HighCmd()
    fill_defaults(stop)
    stop.mode = 2
    stop.gaitType = gait
    stop.speedLevel = speed
    stop.footRaiseHeight = foot_h
    stop.bodyHeight = body_h

    rospy.loginfo("STOP 1.0s")
    end_t = rospy.Time.now() + rospy.Duration.from_sec(1.0)
    while rospy.Time.now() < end_t and not rospy.is_shutdown():
        pub.publish(stop)
        rate.sleep()

    rospy.loginfo("Done.")


if __name__ == "__main__":
    motion_plan()