import argparse
import os
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument(
    '--input-dir', '-i', type=str,
    default='/media/kosuke55/SANDISK/meshdata/ycb_pouring_object/textured_urdf',  # noqa
    help='input urdf directory')
args = parser.parse_args()

input_dir = args.input_dir

paths = list(Path(input_dir).glob('*/base.obj'))
out_dir = Path(input_dir) / 'annotation_obj'
os.makedirs(out_dir, exist_ok=True)
for path in paths:
    out_path = out_dir / path.parent.with_suffix('.obj').name
    shutil.copy(str(path), str(out_path))
    shutil.copy(str(path.with_suffix('.urdf')),
                str(out_path.with_suffix('.urdf')))
    tree = ET.parse(str(out_path.with_suffix('.urdf')))
    root = tree.getroot()
    root[0].find('visual').find('geometry').find(
        'mesh').attrib['filename'] = path.parent.with_suffix('.obj').name
    root[0].find('collision').find('geometry').find(
        'mesh').attrib['filename'] = path.parent.with_suffix('.obj').name
    print(path.parent.with_suffix('.obj').name)
    tree.write(str(out_path.with_suffix('.urdf')),
               encoding='utf-8', xml_declaration=True)
