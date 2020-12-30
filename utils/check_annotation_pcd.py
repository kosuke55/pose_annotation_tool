import argparse

from pathlib import Path

import numpy as np
import open3d as o3d
import skrobot
import trimesh
from hanging_points_generator.generator_utils import load_json

parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument(
    '--input', '-i', type=str,
    help='input annotation directory which has pcd and json.',
    default='/media/kosuke55/SANDISK/meshdata/ycb_hanging_object/real_ycb_annotation')  # noqa
args = parser.parse_args()
annotation_dir = args.input

pcd_paths = list(sorted(Path(annotation_dir).glob('*.pcd')))

pcd_path = pcd_paths[0]
for pcd_path in pcd_paths:
    pcd = o3d.io.read_point_cloud(str(pcd_path))

    paths = list(sorted(Path('./') .glob('*.json')))
    pose_path = pcd_path.with_suffix('.json')

    contact_points_dict = load_json(str(pose_path))
    contact_points = contact_points_dict['contact_points']

    contact_point_sphere_list = []
    for i, cp in enumerate(contact_points):
        contact_point_sphere = skrobot.model.Axis(0.003, 0.05)
        contact_point_sphere.newcoords(
            skrobot.coordinates.Coordinates(pos=cp[0], rot=cp[1:]))
        contact_point_sphere_list.append(contact_point_sphere)

    viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 640))

    trimesh_pc = trimesh.PointCloud(
        np.asarray(pcd.points), np.asarray(pcd.colors))

    pc = skrobot.model.PointCloudLink(trimesh_pc)
    viewer.add(pc)
    for contact_point_sphere in contact_point_sphere_list:
        viewer.add(contact_point_sphere)
    print('Press [q] on the window to move the next object.\n'
          + 'Press [ctrl + c] on the command line to finish.')
    viewer._init_and_start_app()
