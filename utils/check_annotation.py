from pathlib import Path
from hanging_points_generator.generator_utils \
    import check_contact_points

texture = True
urdf_dir = '/media/kosuke55/SANDISK/meshdata/ycb_hanging_object/urdf2'

annotation_dir = '/media/kosuke55/SANDISK/meshdata/ycb_hanging_object/urdf2/annotation_obj'  # noqa

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
