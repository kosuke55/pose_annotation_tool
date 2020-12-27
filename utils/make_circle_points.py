import argparse
import os.path as osp
from pathlib import Path

from hanging_points_generator.generator_utils \
    import save_contact_points_as_annotation_format
from hanging_points_generator.generator_utils import save_contact_points
from hanging_points_generator.generator_utils import check_contact_points
from hanging_points_generator.generator_utils import coords_to_dict
from hanging_points_generator.generator_utils import make_average_coords_list
from skrobot.coordinates import Coordinates


def make_circle_coords(r, interval):
    # reference
    # https://stackoverflow.com/questions/39862709/generate-coordinates-in-grid-that-lie-within-a-circle/42375645  # noqa
    R = r / interval
    X = int(R)  # R is the radius
    coords_list = []
    for x in range(-X, X + 1):
        Y = int((R * R - x * x)**0.5)  # bound for y given x
        for y in range(-Y, Y + 1):
            coords_list.append([x * interval,
                                y * interval])

    return coords_list

parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument(
    '--input', '-i',
    type=str, help='input annotation file',
    default='/media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf/annotation_obj/029_plate.txt')
parser.add_argument(
    '--replace', '-r',
    action='store_true', help='If true, repace annotation file')
parser.add_argument(
        '--radius',
        type=float, help='radius of circle points',
        default=0.1)
parser.add_argument(
        '--interval',
        type=float, help='interval of points',
        default=0.01)
args = parser.parse_args()

input_file = args.input
urdf_file = str(Path(input_file).with_suffix('.urdf'))
if args.replace:
    output_file = input_file
else:
    output_file = osp.splitext(input_file)[0] + '_circle.txt'
radius = args.radius
interval = args.interval

label_list = []
pos_list = []
coords_list = []
with open(input_file)as f:
    for line in f.readlines():
        values = [float(v) for v in line.split()]
        label = int(values[0])
        pos = values[1:4]
        quaternion = values[4:]
        c = Coordinates(pos=pos, rot=quaternion)

        label_list.append(label)
        pos_list.append(pos)
        coords_list.append(c)

print('coords_list', coords_list)
dummy_labels = [0] * len(coords_list)
base_coords = make_average_coords_list(
    coords_list, dummy_labels, average_pos=True)[0]


circle_coords_list = []
dy_dz_list = make_circle_coords(
    r=radius, interval=interval)

for dy, dz in dy_dz_list:
    _c = base_coords.copy_worldcoords().translate([0, dy, dz])
    circle_coords_list.append(_c)

circle_coords_dict = coords_to_dict(
    circle_coords_list,
    urdf_file)

tmp_file = 'tmp.json'
save_contact_points(tmp_file, circle_coords_dict)
check_contact_points(tmp_file, urdf_file)
save_contact_points_as_annotation_format(circle_coords_dict, output_file)
