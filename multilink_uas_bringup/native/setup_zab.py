#!/usr/bin/env python3
"""Fetch the pinned ZaB model and adapt only its world settings for Harmonic."""
import argparse,hashlib,json,shutil,urllib.request,zipfile
from pathlib import Path
import xml.etree.ElementTree as ET
URL='https://zenodo.org/records/17407065/files/ZaB_Model.zip?download=1'
MD5='a0409a2c449f67bf5a80d1ac91714886'
DEFAULT=Path(__file__).resolve().parent.parents[3]/'assets/zab'

def setup(dest,archive=None):
    dest.mkdir(parents=True,exist_ok=True)
    saved=dest/'ZaB_Model.zip'
    if archive:
        if archive.resolve()!=saved.resolve():shutil.copy2(archive,saved)
    elif not saved.exists():
        tmp=dest/'download.partial';urllib.request.urlretrieve(URL,tmp);tmp.replace(saved)
    raw=saved.read_bytes()
    if hashlib.md5(raw).hexdigest()!=MD5:raise RuntimeError('ZaB archive checksum mismatch')
    with zipfile.ZipFile(saved) as z:
        for item in z.infolist():
            if not item.filename.startswith('ZaB_Model/models/'):continue
            rel=Path(item.filename).relative_to('ZaB_Model/models')
            if '..' in rel.parts or rel.is_absolute():raise ValueError('Unsafe archive path')
            target=dest/'models'/rel
            if item.is_dir():target.mkdir(parents=True,exist_ok=True)
            else:
                target.parent.mkdir(parents=True,exist_ok=True)
                with z.open(item) as src,target.open('wb') as out:shutil.copyfileobj(src,out)
    upstream=dest/'models/zab_vehicle.sdf';root=ET.parse(upstream).getroot();root.set('version','1.9');w=root.find('world')
    # Preserve all supplied tunnel meshes, materials, poses and lights. Remove only demo vehicle.
    removed=[]
    for model in w.findall('model'):
        if model.get('name')=='vehicle_blue':w.remove(model);removed.append('vehicle_blue')
    missing=[]
    for include in w.findall('include'):
        uri=include.findtext('uri').strip()
        if uri=='model://newJerseyElement' and not (dest/'models/newJerseyElement').exists():
            missing.append(uri);w.remove(include) # Upstream zip omits this entrance barrier.
        elif uri.startswith('model://') and not (dest/'models'/uri[8:]).exists():
            raise RuntimeError('Missing environment model: '+uri)
    for plugin in w.findall('plugin'):
        if plugin.get('name','').endswith('::TriggeredPublisher'):
            w.remove(plugin);continue # Keyboard commands for the removed ground vehicle.
        plugin.set('filename',plugin.get('filename').replace('libignition-gazebo-','gz-sim-').replace('ignition-gazebo-','gz-sim-').removesuffix('.so'))
        plugin.set('name',plugin.get('name').replace('ignition::gazebo::','gz::sim::'))
    physics=w.find('physics');physics.find('max_step_size').text='0.001'
    oldrate=physics.find('real_time_update_rate')
    if oldrate is not None:physics.remove(oldrate)
    scenes=w.findall('scene');scene=scenes[-1]
    for other in scenes[:-1]:
        for child in other:
            if scene.find(child.tag) is None:scene.append(child)
        w.remove(other)
    for uri in w.findall('.//uri'):
        if uri.text:uri.text=uri.text.strip()
    # A useful starting view near the original vehicle spawn.
    gui=ET.SubElement(w,'gui',fullscreen='0')
    for filename,name in [('MinimalScene','3D View'),('GzSceneManager','Scene Manager'),('InteractiveViewControl','View Control'),('EntityTree','Entity Tree'),('WorldControl','World Control'),('WorldStats','World Statistics')]:
        plugin=ET.SubElement(gui,'plugin',filename=filename,name=name)
        if filename=='MinimalScene':
            ET.SubElement(plugin,'engine').text='ogre2';ET.SubElement(plugin,'scene').text='scene';ET.SubElement(plugin,'camera_pose').text='0 0 2 0 0 -0.35'
        elif filename=='WorldControl':
            ET.SubElement(plugin,'service').text='/world/zab/control'
            ET.SubElement(plugin,'stats_topic').text='/world/zab/stats'
        elif filename=='WorldStats':
            ET.SubElement(plugin,'topic').text='/world/zab/stats'
            for field in ('sim_time','real_time','real_time_factor','iterations'):ET.SubElement(plugin,field).text='true'
    destworld=dest/'zab_harmonic.sdf';ET.indent(root);ET.ElementTree(root).write(destworld,encoding='unicode',xml_declaration=True)
    manifest=dict(source=URL,doi='10.5281/zenodo.17407065',archive_md5=MD5,archive_sha256=hashlib.sha256(raw).hexdigest(),world=str(destworld),world_sha256=hashlib.sha256(destworld.read_bytes()).hexdigest(),environment_includes=len(w.findall('include')),lights=len(w.findall('light')),removed_demo_models=removed,missing_upstream_props_omitted=missing,changes=['Ignition system names migrated to gz-sim','3ms physics step changed to 1ms','Demo vehicle keyboard command plugins removed','Duplicate scene entries merged','GUI starting view added'],geometry_and_textures='Unmodified archive contents',license='See upstream distribution; no additional license inferred')
    (dest/'provenance.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(manifest,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--destination',type=Path,default=DEFAULT);p.add_argument('--archive',type=Path);args=p.parse_args();setup(args.destination,args.archive)
