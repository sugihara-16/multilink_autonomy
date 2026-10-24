"""Restore the archived logs into an empty directory, checking each downloaded part."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

REMOTE = 'uas_gdrive_private:multilink_autonomy_archives/logs-20261020'
FOLDER_ID = '1SNkp_XT7AsO2UyedbRQ1FqM1qJKQNiId'


def remote_command(command, name, *args):
    return ['rclone', command, 'uas_gdrive_private:'+name, *args,
            '--drive-root-folder-id', FOLDER_ID]


def md5(path):
    h = hashlib.md5()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(8*1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def get_json(name):
    return json.loads(subprocess.check_output(remote_command('cat', name)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--destination', type=Path, required=True,
                        help='Empty directory that will receive logs/')
    parser.add_argument('--without-bags', action='store_true',
                        help='Restore archived reports/configurations without the 47 separately stored bags')
    args = parser.parse_args()
    dest = args.destination.resolve()
    if dest.exists() and any(dest.iterdir()):
        parser.error('Destination must be empty')
    dest.mkdir(parents=True, exist_ok=True)
    manifest = get_json('archive_complete.json')
    bags = get_json('existing_bags_verified.json')
    decoder = subprocess.Popen(['zstd','-d','-c'],stdin=subprocess.PIPE,stdout=subprocess.PIPE)
    tar = subprocess.Popen(['tar','-xf','-','-C',str(dest)],stdin=decoder.stdout)
    decoder.stdout.close()
    try:
        with tempfile.TemporaryDirectory(prefix='archive-download-') as tmp:
            for record in manifest['parts']:
                p = Path(tmp)/record['name']
                subprocess.run(remote_command('copyto', record['name'], str(p), '--checksum'), check=True)
                assert p.stat().st_size == record['size'] and md5(p) == record['md5']
                with p.open('rb') as f:
                    shutil.copyfileobj(f,decoder.stdin,8*1024*1024)
                p.unlink()
                print('Restored part',record['name'],flush=True)
        decoder.stdin.close()
        assert decoder.wait() == 0
        assert tar.wait() == 0
        if not args.without_bags:
            for record in bags:
                p = dest/'logs'/record['path']
                assert p.resolve().is_relative_to(dest)
                p.parent.mkdir(parents=True,exist_ok=True)
                subprocess.run(remote_command('copyto', 'existing_bags/'+record['md5']+'.mcap',
                                              str(p), '--checksum'), check=True)
                assert p.stat().st_size == record['size'] and md5(p) == record['md5']
                print('Restored bag',record['path'],flush=True)
    except BaseException:
        decoder.terminate()
        tar.terminate()
        raise
    (dest/'RESTORED.json').write_text(json.dumps(dict(remote=REMOTE,bags_restored=not args.without_bags),indent=2)+'\n')


if __name__ == '__main__':
    main()
