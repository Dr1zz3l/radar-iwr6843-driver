"""ROS bag loading and inspection utilities."""

from pathlib import Path
from typing import Dict, List, Optional, Any
import numpy as np

try:
    import rosbag
    HAS_ROSBAG = True
except ImportError:
    HAS_ROSBAG = False

from .structures import (
    MocapPose,
    MocapAccel,
    AgirosState,
    AgirosOdometry,
    IMUData,
    RadarPointCloud,
    RadarVelocity,
    BagData,
)


def inspect_bag_topics(bag_path: str) -> Dict[str, Any]:
    """Inspect rosbag topics and print their structure.
    
    Args:
        bag_path: Path to the rosbag file
        
    Returns:
        Dictionary with topic information
    """
    if not HAS_ROSBAG:
        raise ImportError("rosbag package not available. Install with: pip install rosbag")

    bag = rosbag.Bag(bag_path)
    
    topics_info = {}
    try:
        type_and_topic_info = bag.get_type_and_topic_info()
        
        print(f"\n{'='*60}")
        print(f"Bag inspection: {bag_path}")
        print(f"Duration: {bag.get_end_time() - bag.get_start_time():.2f} seconds")
        print(f"{'='*60}\n")
        
        print(f"{'Topic':<50} {'Type':<40} {'Count':>8}")
        print("-" * 100)
        
        for topic in sorted(type_and_topic_info.topics.keys()):
            topic_info = type_and_topic_info.topics[topic]
            msg_type = topic_info.msg_type
            msg_count = topic_info.message_count
            
            print(f"{topic:<50} {msg_type:<40} {msg_count:>8}")
            topics_info[topic] = {
                "type": msg_type,
                "count": msg_count,
            }
            
            # Sample first message
            try:
                for _, msg, _ in bag.read_messages(topics=[topic]):
                    topics_info[topic]["sample"] = str(msg)[:200]
                    break
            except Exception as e:
                topics_info[topic]["sample"] = f"Error sampling: {e}"
        
        print(f"\n{'='*60}\n")
        
    finally:
        bag.close()
    
    return topics_info


def _extract_position_and_orientation(pose_msg) -> tuple[np.ndarray, np.ndarray]:
    """Extract position [x,y,z] and orientation [qx,qy,qz,qw] from pose message.
    
    Handles both PoseStamped (msg.pose.position) and Pose (msg.position) formats.
    """
    # Try PoseStamped format first (msg.pose.position)
    if hasattr(pose_msg, 'pose') and hasattr(pose_msg.pose, 'position'):
        pos = np.array([
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z,
        ])
        ori = np.array([
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w,
        ])
    # Fall back to Pose format (msg.position)
    elif hasattr(pose_msg, 'position'):
        pos = np.array([
            pose_msg.position.x,
            pose_msg.position.y,
            pose_msg.position.z,
        ])
        ori = np.array([
            pose_msg.orientation.x,
            pose_msg.orientation.y,
            pose_msg.orientation.z,
            pose_msg.orientation.w,
        ])
    else:
        raise AttributeError(f"Cannot extract position/orientation from {type(pose_msg)}")
    
    return pos, ori


def _extract_vector3(vec) -> np.ndarray:
    """Extract [x, y, z] from a Vector3 message."""
    return np.array([vec.x, vec.y, vec.z])


def _extract_quaternion(quat) -> np.ndarray:
    """Extract [x, y, z, w] from a Quaternion message."""
    return np.array([quat.x, quat.y, quat.z, quat.w])


def _extract_point_cloud_velocities(msg):
    """
    Extract all fields from radar PointCloud2 messages.
    
    The radar publishes PointCloud2 messages with custom fields including velocity.
    - /mmWaveDataHdl/RScanVelocity has 9 fields: x, y, z, velocity, intensity, range, noise, time_cpu_cycles, frame_number
    - /ti_mmwave/radar_scan_pcl_0 has 5 fields: x, y, z, intensity, velocity
    
    Args:
        msg: sensor_msgs/PointCloud2 message
        
    Returns:
        Tuple of (positions, velocities, intensities, ranges, noise, time_cpu_cycles, frame_number)
        Arrays are numpy arrays or None if field not present
    """
    import sensor_msgs.point_cloud2 as pc2
    
    # Get field names to determine which fields are available
    field_names = [field.name for field in msg.fields]
    
    # Initialize lists
    positions = []
    velocities = []
    intensities = []
    ranges = []
    noise_vals = []
    time_cycles = []
    frame_nums = []
    
    # Read all points from the cloud
    for point in pc2.read_points(msg, skip_nans=True):
        # Extract x, y, z (should always be present)
        if 'x' in field_names and 'y' in field_names and 'z' in field_names:
            x_idx = field_names.index('x')
            y_idx = field_names.index('y')
            z_idx = field_names.index('z')
            positions.append([point[x_idx], point[y_idx], point[z_idx]])
        
        # Extract velocity (radial velocity in m/s)
        if 'velocity' in field_names:
            vel_idx = field_names.index('velocity')
            velocities.append(point[vel_idx])
        
        # Extract intensity (reflected power)
        if 'intensity' in field_names:
            int_idx = field_names.index('intensity')
            intensities.append(point[int_idx])
        
        # Extract range (computed from x, y, z)
        if 'range' in field_names:
            range_idx = field_names.index('range')
            ranges.append(point[range_idx])
        
        # Extract noise
        if 'noise' in field_names:
            noise_idx = field_names.index('noise')
            noise_vals.append(point[noise_idx])
        
        # Extract time_cpu_cycles
        if 'time_cpu_cycles' in field_names:
            time_idx = field_names.index('time_cpu_cycles')
            time_cycles.append(point[time_idx])
        
        # Extract frame_number
        if 'frame_number' in field_names:
            frame_idx = field_names.index('frame_number')
            frame_nums.append(point[frame_idx])
    
    # Convert to numpy arrays
    positions = np.array(positions) if len(positions) > 0 else np.array([]).reshape(0, 3)
    velocities = np.array(velocities) if len(velocities) > 0 else None
    intensities = np.array(intensities) if len(intensities) > 0 else None
    ranges = np.array(ranges) if len(ranges) > 0 else None
    noise_vals = np.array(noise_vals) if len(noise_vals) > 0 else None
    time_cycles = np.array(time_cycles, dtype=np.uint32) if len(time_cycles) > 0 else None
    frame_nums = np.array(frame_nums, dtype=np.uint32) if len(frame_nums) > 0 else None
    
    return positions, velocities, intensities, ranges, noise_vals, time_cycles, frame_nums



def load_bag_topics(bag_path: str, verbose: bool = True) -> BagData:
    """Load relevant topics from a rosbag file.
    
    Args:
        bag_path: Path to the rosbag file
        verbose: Print loading progress
        
    Returns:
        BagData object containing all extracted data
    """
    if not HAS_ROSBAG:
        raise ImportError("rosbag package not available. Install with: pip install rosbag")

    bag_path = Path(bag_path)
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag file not found: {bag_path}")

    bag = rosbag.Bag(str(bag_path))
    
    # Initialize storage
    mocap_pose_list = []
    mocap_accel_list = []
    agiros_state_list = []
    agiros_odometry_list = []
    imu_list = []
    radar_pcl_list = []
    radar_velocity_list = []
    
    try:
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()
        duration = end_time - start_time
        
        if verbose:
            print(f"\nLoading rosbag: {bag_path.name}")
            print(f"Duration: {duration:.2f}s")
        
        # Load MoCap Pose
        if verbose:
            print("  Loading /mocap/angrybird2/pose...")
        for _, msg, t in bag.read_messages(topics=["/mocap/angrybird2/pose"]):
            try:
                pos, ori = _extract_position_and_orientation(msg)
                mocap_pose_list.append(MocapPose(
                    timestamp=t.to_sec(),
                    position=pos,
                    orientation=ori,
                ))
            except Exception as e:
                if verbose:
                    print(f"    Error parsing message: {e}")
        
        # Load MoCap Accel (TwistStamped with linear/angular velocity, not accel)
        if verbose:
            print("  Loading /mocap/angrybird2/accel...")
        for _, msg, t in bag.read_messages(topics=["/mocap/angrybird2/accel"]):
            try:
                # Note: Despite the topic name "accel", this is actually TwistStamped
                # containing linear and angular velocity
                linear_vel = _extract_vector3(msg.twist.linear)
                angular_vel = _extract_vector3(msg.twist.angular)
                mocap_accel_list.append(MocapAccel(
                    timestamp=t.to_sec(),
                    linear_acceleration=linear_vel,  # Actually velocity data
                    angular_velocity=angular_vel,
                ))
            except Exception as e:
                if verbose:
                    print(f"    Error parsing message: {e}")
        
        # Load Agiros State (QuadState message, not Odometry)
        if verbose:
            print("  Loading /angrybird2/agiros_pilot/state...")
        for _, msg, t in bag.read_messages(topics=["/angrybird2/agiros_pilot/state"]):
            try:
                pos = _extract_vector3(msg.pose.position)
                ori = _extract_quaternion(msg.pose.orientation)
                vel = _extract_vector3(msg.velocity.linear)
                ang_vel = _extract_vector3(msg.velocity.angular)
                
                # Extract optional fields from QuadState message
                accel = _extract_vector3(msg.acceleration.linear) if hasattr(msg, 'acceleration') else None
                ang_accel = _extract_vector3(msg.acceleration.angular) if hasattr(msg, 'acceleration') and hasattr(msg.acceleration, 'angular') else None
                jerk = _extract_vector3(msg.jerk) if hasattr(msg, 'jerk') else None
                snap = _extract_vector3(msg.snap) if hasattr(msg, 'snap') else None
                acc_bias = _extract_vector3(msg.acc_bias) if hasattr(msg, 'acc_bias') else None
                gyr_bias = _extract_vector3(msg.gyr_bias) if hasattr(msg, 'gyr_bias') else None
                motors = np.array(msg.motors) if hasattr(msg, 'motors') and len(msg.motors) > 0 else None
                
                agiros_state_list.append(AgirosState(
                    timestamp=t.to_sec(),
                    position=pos,
                    velocity=vel,
                    orientation=ori,
                    angular_velocity=ang_vel,
                    acceleration=accel,
                    angular_acceleration=ang_accel,
                    jerk=jerk,
                    snap=snap,
                    acc_bias=acc_bias,
                    gyr_bias=gyr_bias,
                    motors=motors,
                ))
            except Exception as e:
                if verbose:
                    print(f"    Error parsing message: {e}")
        
        # Load Agiros Odometry
        if verbose:
            print("  Loading /angrybird2/agiros_pilot/odometry...")
        for _, msg, t in bag.read_messages(topics=["/angrybird2/agiros_pilot/odometry"]):
            try:
                pos = _extract_vector3(msg.pose.pose.position)
                ori = _extract_quaternion(msg.pose.pose.orientation)
                vel = _extract_vector3(msg.twist.twist.linear)
                ang_vel = _extract_vector3(msg.twist.twist.angular)
                
                agiros_odometry_list.append(AgirosOdometry(
                    timestamp=t.to_sec(),
                    position=pos,
                    velocity=vel,
                    orientation=ori,
                    angular_velocity=ang_vel,
                ))
            except Exception as e:
                if verbose:
                    print(f"    Error parsing message: {e}")
        
        # Load IMU Data
        if verbose:
            print("  Loading /angrybird2/imu...")
        for _, msg, t in bag.read_messages(topics=["/angrybird2/imu"]):
            try:
                accel = _extract_vector3(msg.linear_acceleration)
                ang_vel = _extract_vector3(msg.angular_velocity)
                # Note: msg.orientation exists but contains NaN values in this dataset
                # Only extract if valid (not NaN)
                ori = None
                if hasattr(msg, 'orientation'):
                    quat = _extract_quaternion(msg.orientation)
                    if not np.any(np.isnan(quat)):
                        ori = quat
                
                imu_list.append(IMUData(
                    timestamp=t.to_sec(),
                    linear_acceleration=accel,
                    angular_velocity=ang_vel,
                    orientation=ori,
                ))
            except Exception as e:
                if verbose:
                    print(f"    Error parsing message: {e}")
        
        # Load Radar Point Cloud
        if verbose:
            print("  Loading /ti_mmwave/radar_scan_pcl_0...")
        for _, msg, t in bag.read_messages(topics=["/ti_mmwave/radar_scan_pcl_0"]):
            try:
                # Extract all fields from PointCloud2 (PCL format with x, y, z, intensity, velocity)
                positions, velocities, intensities, ranges, noise, time_cpu_cycles, frame_number = _extract_point_cloud_velocities(msg)
                
                if len(positions) > 0:
                    radar_pcl_list.append(RadarPointCloud(
                        timestamp=t.to_sec(),
                        positions=positions,
                        velocities=velocities,
                        intensities=intensities,
                        ranges=ranges,
                        noise=noise,
                        time_cpu_cycles=time_cpu_cycles,
                        frame_number=frame_number,
                    ))
            except Exception as e:
                if verbose:
                    print(f"    Error parsing message: {e}")
        
        # Load Radar Velocity (Most important radar topic - PointCloud2 format with 9 fields)
        if verbose:
            print("  Loading /mmWaveDataHdl/RScanVelocity...")
        for _, msg, t in bag.read_messages(topics=["/mmWaveDataHdl/RScanVelocity"]):
            try:
                # RScanVelocity is a PointCloud2 message with 9 fields
                positions, velocities, intensities, ranges, noise, time_cpu_cycles, frame_number = _extract_point_cloud_velocities(msg)
                
                if len(positions) > 0:
                    radar_velocity_list.append(RadarVelocity(
                        timestamp=t.to_sec(),
                        positions=positions,
                        velocities=velocities,
                        intensities=intensities,
                        ranges=ranges,
                        noise=noise,
                        time_cpu_cycles=time_cpu_cycles,
                        frame_number=frame_number,
                    ))
            except Exception as e:
                if verbose:
                    print(f"    Error parsing message: {e}")
        
        if verbose:
            print("  Done!\n")
        
    finally:
        bag.close()
    
    # Create BagData object
    bag_data = BagData(
        mocap_pose=mocap_pose_list,
        mocap_accel=mocap_accel_list,
        agiros_state=agiros_state_list,
        agiros_odometry=agiros_odometry_list,
        imu_data=imu_list,
        radar_pcl=radar_pcl_list,
        radar_velocity=radar_velocity_list,
        bag_path=str(bag_path),
        start_time=start_time,
        end_time=end_time,
        duration=duration,
    )
    
    return bag_data
