import argparse

from pathlib import Path
from hanging_points_generator.generator_utils \
    import check_contact_points


parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument(
    '--input-urdf', '-u', type=str,
    help='input urdf directory',
    default='/media/kosuke55/SANDISK/meshdata/ycb_hanging_object/urdf2')  # noqa
parser.add_argument(
    '--input-annotation', '-a', type=str,
    help='input annotation directory',
    default='/media/kosuke55/SANDISK/meshdata/ycb_hanging_object/urdf2/annotation_obj')  # noqa
parser.add_argument(
    '--texture', type=int,
    help='if True, load texture.urdf',
    default=1)
args = parser.parse_args()

urdf_dir = args.input_urdf
annotation_dir = args.input_annotation
texture = args.texture

paths = list(sorted(Path(annotation_dir) .glob('*.json')))
for path in paths:
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
            average_pos=False)
    except KeyboardInterrupt:
        pass
