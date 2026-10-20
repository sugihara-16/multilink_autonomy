"""Reuse persistent raw sensor bags without allowing experiment cleanup to delete them."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent/'logs/zab-local-bags'
def link_local_chunk(label,name,destination,expected_md5):
 """Link a previously verified original; caller still verifies its checksum."""
 manifest=ROOT/label/'verified.json'
 if not manifest.exists():return False
 record=json.loads(manifest.read_text())['files'].get(name)
 if record is None or record['md5']!=expected_md5:return False
 source=ROOT/label/'rosbag'/name
 if not source.is_file() or source.stat().st_size!=record['bytes']:return False
 destination=Path(destination)
 assert destination.absolute()!=source.absolute(),'Never replace a persistent original'
 if destination.is_symlink() or destination.exists():destination.unlink()
 destination.symlink_to(source)
 return True
