"""
create g2o file format from KITTI poses.
"""

import os
import csv
import numpy as np
import transformations as tr

def csv_read_matrix(file_path, delim=',', comment_str="#"):
    """
    directly parse a csv-like file into a matrix
    :param file_path: path of csv file (or file handle)
    :param delim: delimiter character
    :param comment_str: string indicating a comment line to ignore
    :return: 2D list with raw data (string)
    """
    if hasattr(file_path, 'read'):  # if file handle
        generator = (line for line in file_path
                     if not line.startswith(comment_str))
        reader = csv.reader(generator, delimiter=delim)
        mat = [row for row in reader]
    else:
        if not os.path.isfile(file_path):
            raise FileNotFoundError(f"{file_path} file doesn't exist")
        with open(file_path) as f:
            generator = (line for line in f
                         if not line.startswith(comment_str))
            reader = csv.reader(generator, delimiter=delim)
            mat = [row for row in reader]
    return mat


def read_kitti_poses_file(file_path):
    """
    parses pose file in KITTI format (first 3 rows of SE(3) matrix per line)
    :param file_path: the trajectory file path (or file handle)
    :return: trajectory.PosePath3D
    """
    raw_mat = csv_read_matrix(file_path, delim=" ", comment_str="#")
    error_msg = ("KITTI pose files must have 12 entries per row "
                 "and no trailing delimiter at the end of the rows (space)")
    if not raw_mat or (len(raw_mat) > 0 and len(raw_mat[0]) != 12):
        raise Exception(error_msg)
    try:
        mat = np.array(raw_mat).astype(float)
    except ValueError:
        raise Exception(error_msg)
    
    poses = [np.array([[r[0], r[1], r[2], r[3]],
                       [r[4], r[5], r[6], r[7]],
                       [r[8], r[9], r[10], r[11]],
                       [0, 0, 0, 1]]) for r in mat]
    return poses


def write_g2o_poses_file(file_path, poses):
    id = 0
    with open(file_path, 'w') as file:
        file.write(f"# VERTEX_SE3:QUAT_TIME id x y z qx qy qz qw sec nsec\n")
        for pose in poses:
            xyz = pose[0:3,3]
            q = tr.quaternion_from_matrix(pose[0:3,0:3])
            file.write(f"VERTEX_SE3:QUAT_TIME {id} {xyz[0]} {xyz[1]} {xyz[2]} {q[1]} {q[2]} {q[3]} {q[0]} sec nsec\n")
            id = id + 1

# Read poses from file
kitti_file_path = "/home/nived/results/latest/exp21_outside_building.txt"
poses_kitti = read_kitti_poses_file(kitti_file_path)
print(f"Read {len(poses_kitti)} poses from file.")

# write as g2o file
g2o_file_path = "/home/nived/results/latest/exp21_outside_building.g2o"
write_g2o_poses_file(g2o_file_path, poses_kitti)
