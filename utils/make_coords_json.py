import argparse
from pathlib import Path

import numpy as np

from hanging_points_generator.generator_utils import save_json
from skrobot.coordinates.math import quaternion2matrix

parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)

parser.add_argument(
    '--input-dir', '-i', type=str,
    help='input_dir',
    # default='/home/kosuke55/catkin_ws/src/pose_annotation_tool/annotation_obj')  # noqa
    default='/media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf/annotation_obj')  # noqa

args = parser.parse_args()

paths = list(sorted(Path(args.input_dir).glob('*.txt')))
for path in paths:
    input_file = str(path)
    print(input_file)

    contact_points = {'contact_points': [],
                      'labels': [],
                      'urdf_file': str(path.with_suffix('.urdf'))}
    with open(input_file)as f:
        for line in f.readlines():
            values = [float(v) for v in line.split()]
            label = int(values[0])
            pos = values[1:4]
            quaternion = values[4:]
            matrix = quaternion2matrix(quaternion)

            contact_points['contact_points'].append(
                np.vstack([pos, matrix]).tolist())
            contact_points['labels'].append(label)
            print(line)

    save_json(str(path.with_suffix('.json')), contact_points)
    # print(lines)
