import argparse

from pathlib import Path
from hanging_points_generator.generator_utils \
    import check_contact_points


parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument(
    '--input-urdf', '-u', type=str,
    help='input urdf directory',
    # default='/media/kosuke55/SANDISK/meshdata/ycb_hanging_object/urdf2')  # noqa
    default='/media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf')  # noqa
parser.add_argument(
    '--input-annotation', '-a', type=str,
    help='input annotation directory',
    # default='/media/kosuke55/SANDISK/meshdata/ycb_hanging_object/urdf2/annotation_obj')  # noqa
    default='/media/kosuke55/SANDISK/meshdata/ycb_pouring_object_16/textured_urdf/annotation_obj')  # noqa
parser.add_argument(
    '--texture', type=int,
    help='if True, load texture.urdf',
    default=1)
parser.add_argument('--large-axis', '-la', action='store_true',
                    help='use large axis as visulaizing marker')
parser.add_argument('--keyword', '-k', type=str,
                    help='skip files that do not inculude this keyword. '
                    'this option works when input is a directory. ',
                    default=None)
args = parser.parse_args()

urdf_dir = args.input_urdf
annotation_dir = args.input_annotation
texture = args.texture
large_axis = args.large_axis
keyword = args.keyword

paths = list(sorted(Path(annotation_dir) .glob('*.json')))
for path in paths:
    if keyword is not None:
        if keyword not in str(path):
            continue
    print(path)
    pose = str(path)
    if texture:
        urdf = str(Path(urdf_dir) / path.stem / 'textured.urdf')
    else:
        urdf = str(path.with_suffix('.urdf'))

    try:
        check_contact_points(
            pose,
            urdf,
            cluster_min_points=1,
            use_filter_penetration=False,
            inf_penetration_check=False,
            align=False,
            average=False,
            average_pos=False,
            large_axis=large_axis)
    except KeyboardInterrupt:
        pass
