import argparse
import os
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
    X = int(R) + 1
    coords_list = []
    for x in range(-X, X + 1):
        Y = int(((R + 1) ** 2 - x ** 2)**0.5)  # bound for y given x
        for y in range(-Y, Y + 1):
            if x ** 2 + y ** 2 < (R + 0.5) ** 2:
                coords_list.append([x * interval, y * interval])

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
parser.add_argument(
    '--gui', '-g',
    action='store_true',
    help='Visualize geenrated points')
args = parser.parse_args()

input_file = args.input
urdf_file = str(Path(input_file).with_suffix('.urdf'))
if osp.isfile(urdf_file):
    object_file = urdf_file
else:
    object_file = str(Path(input_file).with_suffix('.pcd'))
    if not osp.isfile(object_file):
        raise ValueError('Neither urdf nor pcd can be found.')

if args.replace:
    output_file = input_file
else:
    output_file = osp.splitext(input_file)[0] + '_circle.txt'

radius = args.radius
interval = args.interval
gui = args.gui

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

print('the number of exsiting points: ',
      len(coords_list))
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
    object_file)

if gui:
    tmp_file = 'tmp.json'
    save_contact_points(tmp_file, circle_coords_dict)
    check_contact_points(tmp_file, object_file)
    os.remove(tmp_file)

save_contact_points_as_annotation_format(circle_coords_dict, output_file)
