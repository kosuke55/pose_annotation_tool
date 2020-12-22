import os
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

paths = list(Path('./').glob('*/base.obj'))
out_dir = 'annotation_obj'
os.makedirs(out_dir, exist_ok=True)
for path in paths:
    out_path = out_dir / path.parent.with_suffix('.obj')
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
