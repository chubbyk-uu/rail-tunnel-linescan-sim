"""Export a generated demo and its runtime dependencies as a relocatable directory.

Historical source_file labels remain provenance only. Runtime file/URI references
are copied and made relative; changed JSON dependency hashes are propagated.
"""
import argparse
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import yaml
from .session import sha256_file


def export_demo(demo, output):
    demo=Path(demo).resolve();output=Path(output).resolve()
    if output.exists():raise ValueError('refuse to overwrite demo bundle')
    config=yaml.safe_load((demo/'capture.yaml').read_text())
    if not config['truth'].get('optical_key'):
        raise ValueError('bundle requires a prepared persistent optical key')
    output.mkdir(parents=True);assets=output/'assets';assets.mkdir()
    copied={};hashes={}

    def copy_asset(source):
        source=Path(source).resolve()
        if source in copied:return copied[source]
        if not source.is_file():raise ValueError(f'missing demo dependency: {source}')
        target=assets/f'{len(copied):04d}_{source.name}'
        copied[source]=target
        old_hash=sha256_file(source)
        if source.suffix=='.json':
            data=json.loads(source.read_text())
            def relocate(node):
                if isinstance(node,dict):
                    # Guard manifests refer to the defect hash: copy that dependency first.
                    if 'defects' in node:node['defects']=relocate(node['defects'])
                    if isinstance(node.get('file'),str):
                        child=copy_asset(source.parent/node['file'])
                        node['file']=child.name
                        if 'sha256' in node:node['sha256']=sha256_file(child)
                    for key,value in list(node.items()):
                        # Normalized source.bin is the runtime input. Original scanned
                        # image names are provenance labels, not recipe dependencies.
                        if key not in ('file','sha256','defects','input_channels'):node[key]=relocate(value)
                elif isinstance(node,list):return [relocate(x) for x in node]
                elif isinstance(node,str):return hashes.get(node,node)
                return node
            data=relocate(data)
            target.write_text(json.dumps(data,indent=2)+'\n')
        else:shutil.copyfile(source,target)
        hashes[old_hash]=sha256_file(target)
        return target

    scene=copy_asset(demo/config['render']['optical_scene'])
    config['render']['optical_scene']=str(scene.relative_to(output))
    (output/'capture.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    world=ET.parse(demo/'world/world.sdf')
    for tag in ('uri','albedo_map','normal_map','roughness_map','metalness_map'):
        for element in world.getroot().iter(tag):
            if not element.text:continue
            value=element.text.removeprefix('file://')
            if '://' in value:raise ValueError(f'external world resource is not bundled: {value}')
            target=copy_asset(demo/'world'/value)
            element.text='../'+str(target.relative_to(output))
    (output/'world').mkdir();ET.indent(world)
    world.write(output/'world/world.sdf',encoding='unicode',xml_declaration=True)
    for name in ('calibration.json','gui.config'):
        if not (demo/name).is_file():raise ValueError(f'missing prepared demo {name}')
        shutil.copyfile(demo/name,output/name)
    manifest=dict(schema='ssb.demo_bundle.v1',
                  files={str(p.relative_to(output)):sha256_file(p) for p in output.rglob('*') if p.is_file()},
                  note='Contains private simulation truth for generating captures; reconstruction uses session observable data.')
    (output/'bundle.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--demo',required=True);p.add_argument('--output',required=True)
    a=p.parse_args();manifest=export_demo(a.demo,a.output)
    print(json.dumps({'files':len(manifest['files']),'output':str(Path(a.output).resolve())}))


if __name__=='__main__':main()
