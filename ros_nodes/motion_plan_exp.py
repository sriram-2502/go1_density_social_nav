from pathlib import Path
import argparse

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation, patches
from datetime import datetime

from density_utils.controllers import density_feedback_control
from density_utils.density import Obstacle
from density_utils.utils import plot_goal, plot_obstacle, plot_start
from density_utils.utils.timing import TimedBlock

import rospy
import tf
import rosbag
from autoware_msgs.msg import DetectedObjectArray
from geometry_msgs.msg import PointStamped, PoseStamped, PointStamped
from unitree_legged_msgs.msg import HighCmd
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion



# global variable to store subscribed data
latest_detections = []
pose = []

HIGHLEVEL = 0  # common for unitree_ros_to_real; verify via rostopic echo /high_cmd

def pose_callback(msg):
    global pose
    pose = np.array([msg.pose.position.x-0.12, msg.pose.position.y, msg.pose.orientation.z])
    #print(msg.pose.position)

def detection_callback(msg):
    global latest_detections
    latest_detections = msg.objects
    #rospy.loginfo(f"Received {len(latest_detections)} detections")

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


def _angle_wrap(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def _triangle_points(center, heading, size):
    c = np.array(center, dtype=float)
    forward = np.array([np.cos(heading), np.sin(heading)])
    right = np.array([np.cos(heading + np.pi / 2.0), np.sin(heading + np.pi / 2.0)])
    tip = c + size * 1.3 * forward
    left = c - size * 0.9 * forward + size * 0.6 * right
    right_pt = c - size * 0.9 * forward - size * 0.6 * right
    return np.stack([tip, left, right_pt], axis=0)


def calculate_fov_points(position, heading, fov_angle, cam_range):
    half_fov = fov_angle / 2.0
    left_angle = heading - half_fov
    right_angle = heading + half_fov
    left_point = (
        position[0] + cam_range * np.cos(left_angle),
        position[1] + cam_range * np.sin(left_angle),
    )
    right_point = (
        position[0] + cam_range * np.cos(right_angle),
        position[1] + cam_range * np.sin(right_angle),
    )
    return left_point, right_point


def detect_sensed_obstacles(pos, heading, obstacles, cam_range, fov_angle):
    """Range + FOV filtering based on robot heading."""
    pos = np.asarray(pos, dtype=float)
    sensed = []
    for obs in obstacles:
        rel = obs.center - pos
        dist = np.linalg.norm(rel)
        if dist > cam_range:
            continue
        if fov_angle < 2.0 * np.pi:
            angle_to_obs = np.arctan2(rel[1], rel[0])
            if abs(_angle_wrap(angle_to_obs - heading)) > fov_angle / 2.0:
                continue
        sensed.append((dist, obs))
    sensed.sort(key=lambda item: item[0])
    return [obs for _, obs in sensed]


def sample_obstacle_boundary(obs, num=120):
    """Sample obstacle boundary for visualization."""
    theta = np.linspace(0.0, 2.0 * np.pi, num=num, endpoint=True)
    c = np.cos(theta)
    s = np.sin(theta)
    p = float(obs.p)
    x = np.sign(c) * (np.abs(c) ** (2.0 / p))
    y = np.sign(s) * (np.abs(s) ** (2.0 / p))
    pts = np.stack([x, y], axis=1) * obs.r1
    if obs.scale is not None:
        pts = pts * np.asarray(obs.scale, dtype=float)[None, :]
    if obs.angle:
        ca = np.cos(obs.angle)
        sa = np.sin(obs.angle)
        rot = np.array([[ca, -sa], [sa, ca]])
        pts = pts @ rot.T
    return pts + obs.center[None, :]


def motion_plan():
    parser = argparse.ArgumentParser()
    parser.add_argument("--save-gif", action="store_true", help="Save animation as GIF.")
    args = parser.parse_args()

    #subscribe to ROS topics
    rospy.init_node('motion_plan')
    rospy.Subscriber("/detections", DetectedObjectArray, detection_callback)
    rospy.Subscriber("/camera_pose", PoseStamped, pose_callback)
    topic = rospy.get_param("~topic", "/high_cmd")
    hz = float(rospy.get_param("~hz", 10.0))

    #listen to ROS transforms
    listener = tf.TransformListener()

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    pub = rospy.Publisher(topic, HighCmd, queue_size=10)
    rate = rospy.Rate(hz)
    outBag = rosbag.Bag(f'vel_{timestamp}.bag', 'w')

    stand_time = float(2.0)

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

    dt = 1/hz
    steps = 400
    alpha = 0.4
    ctrl_multiplier = 1.0
    rad_from_goal = 0.8
    stop_tol = min(0.005, rad_from_goal)
    stop_steps = 500
    stop_when_stable = True
    q_lqr = 4.0
    r_lqr = 1.0
    saturation = 4.0
    k_heading = 6.0
    v_max = .3
    v_min = .1
    v_ignore = 0.02
    omega_max = 1.0
    omega_min = .2
    omega_ignore = 0.05
    walk_time  = steps/dt
    obs_buffer_radius = .5


    agent_radius = 0.1
    goal = np.array([4, 0])

    r1 = 1
    sensing_margin = 0.75
    r2 = r1 + sensing_margin

    state = np.array([pose[0], pose[1], pose[2]], dtype=float)
    tilde_prev = pose[2]
    traj = [state.copy()]

    markpub = rospy.Publisher("/detections_markers", MarkerArray, queue_size=10)
    marker_array = MarkerArray()
    marker_array_buffer = MarkerArray()
    markBag = rosbag.Bag(f'markers_{timestamp}.bag', 'w')

    obstacles = []
    obstacles_buffer = []
    detections_prev = []
    controls = []

    stop_count = 0
    rospy.loginfo("BEGIN WALK %.1fs @ %.1fHz", walk_time, hz)
    end_t = rospy.Time.now() + rospy.Duration.from_sec(walk_time)
    while rospy.Time.now() < end_t and not rospy.is_shutdown():
        heading = state[2]

        try:
            (trans,rot) = listener.lookupTransform('world', 'camera', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        yaw = np.arctan2(2*(rot[0]*rot[1]-rot[2]*rot[3]), 1-2*(rot[1]**2 + rot[3]**2))

        #print(f"Number of detections: {len(latest_detections)}")
        marker_array_current = MarkerArray()
        obstacles_current = []
        for det in latest_detections:
            point = PointStamped()
            point.header.frame_id = det.header.frame_id
            point.header.stamp = rospy.Time(0)
            point.point = det.pose.position

            point_velocity = PointStamped()
            point_velocity.header.frame_id = det.header.frame_id
            point_velocity.header.stamp = rospy.Time(0)
            point_velocity.point = det.velocity.linear

            transformed_point = listener.transformPoint('world', point)
            transformed_velocity = listener.transformPoint('world', point_velocity)
            
            x = transformed_point.point.x
            y = transformed_point.point.y
            z = transformed_point.point.z

            vx = transformed_velocity.point.x
            vy = transformed_velocity.point.y
            vz = transformed_velocity.point.z

            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = rospy.Time(0)
            marker.ns = "obstacles"
            marker.id = det.id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration.from_sec(0.1)

            marker.pose.position = transformed_point.point
            marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            marker.scale.x = det.dimensions.x
            marker.scale.y = det.dimensions.y
            marker.scale.z = det.dimensions.z

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5

            bboxx = det.dimensions.x
            bboxy = det.dimensions.y

            rx = np.sqrt(bboxx**2 + bboxy*bboxx) #radius of elipse enclosing object in the x direction
            ry = np.sqrt(bboxy**2 + bboxx*bboxy) #radius of elipse enclosing object in the y direction

            #if np.sqrt((x-pose[0])**2 + (y-pose[1])**2) < min(rx, ry) + sensing_margin:
            #    print("robot within sensing margin")
        
            obstacles_current.append(Obstacle(center=np.array([x, y]), r1=r1, r2=r2, p=1, scale=np.array([rx, ry]), angle=np.deg2rad(yaw)))
            marker_array_current.markers.append(marker)
            #print(f"Object at {x:.2f}, {y:.2f}, {z:.2f}")
            #print(f"Object size rx={rx:.2f}, ry={ry:.2f}")
        #print(f"Number of obstacles: {len(obstacles)}")

        obstacles = obstacles_current.copy()
        marker_array.markers = marker_array_current.markers.copy()

        det_cur_dict = {det.id: det for det in latest_detections}
        det_prev_dict = {det.id: det for det in detections_prev}

        removed_det_id = det_prev_dict.keys() - det_cur_dict.keys()
        removed_det = [det_prev_dict[id] for id in removed_det_id]


        for det in removed_det:
            point = PointStamped()
            point.header.frame_id = det.header.frame_id
            point.header.stamp = det.header.stamp
            point.point = det.pose.position

            point_velocity = PointStamped()
            point_velocity.header.frame_id = det.header.frame_id
            point_velocity.header.stamp = det.header.stamp
            point_velocity.point = det.velocity.linear

            transformed_point = listener.transformPoint('world', point)
            transformed_velocity = listener.transformPoint('world', point_velocity)
            
            x = transformed_point.point.x
            y = transformed_point.point.y
            z = transformed_point.point.z

            vx = transformed_velocity.point.x
            vy = transformed_velocity.point.y
            vz = transformed_velocity.point.z

    
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = det.header.stamp
            marker.ns = "obstacles"
            marker.id = det.id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration.from_sec(0.1)

            marker.pose.position = transformed_point.point
            marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            marker.scale.x = det.dimensions.x
            marker.scale.y = det.dimensions.y
            marker.scale.z = det.dimensions.z

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5

            bboxx = det.dimensions.x
            bboxy = det.dimensions.y

            rx = np.sqrt(bboxx**2 + bboxy*bboxx) #radius of elipse enclosing object in the x direction
            ry = np.sqrt(bboxy**2 + bboxx*bboxy) #radius of elipse enclosing object in the y direction
            
            #if np.sqrt((x-pose[0])**2 + (y-pose[1])**2) < max(rx, ry) + sensing_margin:
            #    print("robot within sensing margin")

            if np.sqrt((x-pose[0])**2 + (y-pose[1])**2) < obs_buffer_radius:
                obstacles_buffer.append(Obstacle(center=np.array([x, y]), r1=r1, r2=r2, p=1, scale=np.array([rx, ry]), angle=np.deg2rad(yaw)))
                marker_array_buffer.markers.append(marker)

        detections_prev = latest_detections.copy()

        obstacles_buffer = [obs for obs in obstacles_buffer if np.sqrt((obs.center[0]-pose[0])**2 + (obs.center[1]-pose[1])**2) < obs_buffer_radius]
        marker_array_buffer.markers = [marker for marker in marker_array_buffer.markers if np.sqrt((marker.pose.position.x-pose[0])**2 + (marker.pose.position.y-pose[1])**2) < obs_buffer_radius]

        obstacles.extend(obstacles_buffer)
        marker_array.markers.extend(marker_array_buffer.markers)
                                
        markpub.publish(marker_array)
        markBag.write("/detections_markers", marker_array)


        inflated_obstacles = [
            Obstacle(
                center=obs.center,
                r1=obs.r1 + agent_radius,
                r2=obs.r2 + agent_radius,
                p=obs.p,
                scale=obs.scale,
                angle=obs.angle,
            )
            for obs in obstacles
        ]


        dist = np.linalg.norm(state[:2] - goal)
     
        u = density_feedback_control(
            state[:2],
            goal,
            alpha,
            inflated_obstacles,
            ctrl_multiplier=ctrl_multiplier,
            rad_from_goal=rad_from_goal,
            q_lqr=q_lqr,
            r_lqr=r_lqr,
            dt=dt,
            saturation=saturation,
        )
        
        v = float(np.linalg.norm(u))
        v = min(v, v_max)
        tilde = float(np.arctan2(u[1], u[0]))
        tilde_dot = _angle_wrap(tilde - tilde_prev) / dt
        tilde_prev = tilde
        omega = tilde_dot - k_heading * _angle_wrap(state[2] - tilde)
        if abs(omega) < omega_ignore:
            omega_saturated = 0
        elif omega > 0:
            omega_saturated = float(np.clip(omega, omega_min, omega_max))
        elif omega < 0:
            omega_saturated = float(np.clip(omega, -omega_max, -omega_min))

        if abs(v) < v_ignore:
            v_saturated = 0
        elif v > 0:
            v_saturated = float(np.clip(v, v_min, v_max))
        elif v < 0:
            v_saturated = float(np.clip(v, -v_max, -v_min))
        

        #print(omega, omega_saturated)

        controls.append([v_saturated, omega_saturated])
        rospy.loginfo(f"Control: v(sent)={v_saturated:.2f}, v(calculated)={v:.2f}, omega(sent)={omega_saturated:.2f}, omega(calculated)={omega:.2f}, dist={dist:.2f}, obsticles={len(obstacles)}")

        walk.velocity[0] = v_saturated
        walk.velocity[1] = 0
        walk.yawSpeed = omega_saturated

        pub.publish(walk)
        outBag.write(topic, walk, rospy.Time.now())

        rate.sleep()

        state = np.array([pose[0], pose[1], pose[2]], dtype=float)


        traj.append(state.copy())
        if stop_when_stable:
            if dist < stop_tol:
                stop_count += 1
                if stop_count >= stop_steps:
                    break
            else:
                stop_count = 0

        if np.linalg.norm(state[:2] - goal) < rad_from_goal:
            break

    # Phase 3: STOP (walk mode, zero velocities)
    stop = HighCmd()
    fill_defaults(stop)
    stop.mode = 2
    stop.gaitType = gait
    stop.speedLevel = speed
    stop.footRaiseHeight = foot_h
    stop.bodyHeight = body_h

    markBag.close()
    outBag.close()


    rospy.loginfo("STOP 3.")
    end_t = rospy.Time.now() + rospy.Duration.from_sec(3.0)
    while rospy.Time.now() < end_t and not rospy.is_shutdown():
        pub.publish(stop)
        rate.sleep()

    rospy.loginfo("Done.")



if __name__ == "__main__":
    motion_plan()







