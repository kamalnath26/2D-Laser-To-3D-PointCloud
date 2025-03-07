import numpy as np
import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh
import rospy
import rosbag

import argparse
import sys
import time
from os import path

# config
use_left_side = False # which side of LIDAR sensor
bag_cutoff_time = None # rospy.Duration(120) # seconds
angle_increment = None
lidar_min_angle = np.radians(5) # degrees
lidar_max_angle = np.radians(180)

def parse_bag_file(bag_filename):
    bag = rosbag.Bag(bag_filename)
    start_t = None

    ranges = []
    servo_angles = []
    range_length = 455

    raw_lidar_minmax = None

    print(f"Reading bag file {bag_filename}...")
    read_start = time.perf_counter()
    n_messages = 0

    for topic, msg, t in bag.read_messages(topics=['/scan', '/servo/angle']):
        if start_t is None:
            start_t = t

        if bag_cutoff_time is not None and (t - start_t) > bag_cutoff_time:
            break

        if topic == '/servo/angle':
            servo_angles.append(msg.data)
            n_messages += 1
        elif topic == '/scan':
            global angle_increment
            if angle_increment is None:
                angle_increment = msg.angle_increment
            if raw_lidar_minmax is None:
                raw_lidar_minmax = (msg.angle_min, msg.angle_max)

            single_ranges = np.array(msg.ranges)
            if len(single_ranges) < range_length:
                single_ranges = np.pad(single_ranges, (0, range_length - len(single_ranges)))
            else:
                single_ranges = single_ranges[0:range_length]
            ranges.append(single_ranges)
            n_messages += 1

        # ignore other topics

    bag.close()
    read_elapsed = time.perf_counter() - read_start
    print(f"Read {n_messages} messages from {bag_filename} in {read_elapsed:.3f} seconds")

    # post process - convert to numpy arrays, compute lidar angle sweep
    servo_angles =  np.array(servo_angles)
    ranges = np.stack(ranges)

    raw_lidar_angles = np.arange(raw_lidar_minmax[0], raw_lidar_minmax[0] + range_length * angle_increment, angle_increment)

    # trim to same length
    num_complete_readings = min(servo_angles.shape[0], ranges.shape[0])
    servo_angles = servo_angles[0:num_complete_readings]
    ranges = ranges[0:num_complete_readings]

    print("read_bag_file")
    print(f"angle inc: {angle_increment=} raw lidar angles: len {len(raw_lidar_angles)} end {raw_lidar_angles[-1]}")
    print(f"{ranges.shape=}, {raw_lidar_angles.shape=}")

    return ranges, servo_angles, raw_lidar_angles



def preprocess(data, raw_lidar_angles, mount_style):
    # trim off readings "behind" the lidar / where the bracket obstructs it
    if use_left_side:
        print("min", lidar_min_angle / angle_increment)
        print("max", lidar_max_angle / angle_increment)

        min_idx = int(round(lidar_min_angle / angle_increment))
        max_idx = int(round(lidar_max_angle / angle_increment))
    elif mount_style == MOUNT_PERPENDICULAR: # use right side only when perpendicular mounting
        end_angle = len(raw_lidar_angles) * angle_increment
        print("min", (end_angle - lidar_min_angle) / angle_increment)
        print("max", (end_angle - lidar_max_angle) / angle_increment)

        min_idx = int(round((end_angle - lidar_max_angle) / angle_increment))
        max_idx = int(round((end_angle  - lidar_min_angle) / angle_increment))

    print(f"preprocess: {min_idx=} {max_idx=}")
    ranges = data[:, min_idx:max_idx]

    return np.nan_to_num(ranges), raw_lidar_angles[min_idx:max_idx]

MOUNT_PERPENDICULAR = 'perpendicular'
MOUNT_PARALLEL = 'parallel'

def transform_perpendicular(ranges, servo_angles, lidar_angles):
    # theta = lidar angle, rho = servo angle

    if use_left_side:
        # convert to right-handed angle (increase from z axis downwards)
        thetas = np.pi - lidar_angles # already in radians
    else:
        # already right handed
        thetas = lidar_angles - np.pi
    print(f"{ranges.shape=}, {thetas.shape=}")

    # servo angles in radians
    rhos = np.radians(180 - servo_angles) # move from left handed angle to right-handed angle (increase CCW)
    # reshape rhos to vertical
    rhos = rhos.reshape(-1, 1)

    # horizontal component of range (broadcast multiplication)
    ranges_xy = ranges * np.sin(thetas)
    ranges_z = ranges * np.cos(thetas)

    x = ranges_xy * np.cos(rhos)
    y = ranges_xy * np.sin(rhos)

    x = x.flatten().reshape(-1, 1)
    y = y.flatten().reshape(-1, 1)
    z = ranges_z.flatten().reshape(-1, 1)

    return x, y, z

def transform_parallel(ranges, servo_angles, lidar_angles):
    # rho is lidar angle, theta is servo angle

    # convert to right-handed angle (increase from x axis CCW)
    rhos = np.pi - lidar_angles

    # servo angles from in radians
    thetas =  np.radians(servo_angles - 90) # already right handed (increases CCW) but 0 is horizontal, we want 0 to be vertical
    # make thetas column, since rows represent servo position
    thetas = thetas.reshape(-1, 1)

    x = ranges * np.cos(rhos) # x component on lidar plane
    ranges_yz = ranges * np.sin(rhos) # y component on lidar plane

    # (rotation is about x axis, so we don't need to transform)
    y = ranges_yz * np.sin(thetas) # transform by servo angle
    z = ranges_yz * np.cos(thetas) # transform by servo angle

    x = x.flatten().reshape(-1, 1)
    y = y.flatten().reshape(-1, 1)
    z = z.flatten().reshape(-1, 1)

    return x, y, z


def ranges_to_point_cloud(raw_ranges, servo_angles, raw_lidar_angles, mount_style=MOUNT_PERPENDICULAR):
    ranges, lidar_angles = preprocess(raw_ranges, raw_lidar_angles, mount_style)

    if mount_style == MOUNT_PERPENDICULAR:
        x, y, z = transform_perpendicular(ranges, servo_angles, lidar_angles)
    elif mount_style == MOUNT_PARALLEL:
        x, y, z = transform_parallel(ranges, servo_angles, lidar_angles)
    else:
        raise ValueError('Unsupported mount style')

    xyz = np.hstack((x, y, z))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    print(f"Created point cloud with {xyz.shape[0]} points.")

    return pcd

def visualize_point_cloud(pcd):
    # pcd.estimate_normals()
    # pcd_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd)
    # pcd_mesh.paint_uniform_color([1, 0, 0])
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
    o3d.visualization.draw_geometries([mesh_frame, pcd])

def write_pcd_to_bag(pcd, outfile):
    rospy.init_node("scan2cloud")
    output_bag = rosbag.Bag(outfile, 'w')

    try:
        ros_pcd = orh.o3dpc_to_rospc(pcd)
        output_bag.write('point_cloud', ros_pcd)
    finally:
        output_bag.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_file')
    parser.add_argument('--mount', default=MOUNT_PERPENDICULAR)
    parser.add_argument('--test', help='Test with spherical data', action='store_true')
    parser.add_argument('--lidar-right', '-r', help='Use right side of LIDAR (180-355)', action='store_true')
    parser.add_argument('--save-ply', help='Save Open3D point cloud as .ply file', action='store_true')
    parser.add_argument('--output-bag', '-o',
                        help='Output a ROS PointCloud2 as a bag file',
                        action='store_true')

    args = parser.parse_args(sys.argv[1:])
    mount_style = args.mount
    use_left_side = not args.lidar_right

    if mount_style == MOUNT_PERPENDICULAR:
        lidar_min_angle = np.radians(5) # degrees
        lidar_max_angle = np.radians(180)
    elif mount_style == MOUNT_PARALLEL:
        lidar_min_angle = np.radians(90) # degrees
        lidar_max_angle = np.radians(270)

    if args.test:
        print("Testing with spherical data. Ignoring bag file argument.")
        raw_ranges = np.random.rand(340, 455) * 0.2 + 6.0
        raw_lidar_angles, angle_increment = np.linspace(0, 2*np.pi, 455, retstep=True)
        servo_angles = np.linspace(-170, 170, 340)
    else:
        raw_ranges, servo_angles, raw_lidar_angles = parse_bag_file(args.bag_file)

    # Convert LIDAR data to Open3d point cloud
    # (this is the magic)
    pcd = ranges_to_point_cloud(raw_ranges, servo_angles, raw_lidar_angles, mount_style)

    if args.save_ply:
        pcd_filename = args.bag_file.replace(".bag", ".ply")
        o3d.io.write_point_cloud(pcd_filename, pcd)
        print(f"Saved point cloud to {pcd_filename}")

    if args.output_bag:
        output_filename = f"cloud_{path.basename(args.bag_file)}"
        write_pcd_to_bag(pcd, output_filename)

    visualize_point_cloud(pcd)


