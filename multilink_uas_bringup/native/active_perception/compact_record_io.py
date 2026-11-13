"""Read numerical records before or after lossless storage compaction."""
import gzip
from pathlib import Path


def record_path(path):
    path=Path(path)
    return path if path.exists() else path.with_name(path.name+'.gz')


def open_record(path):
    path=record_path(path)
    return gzip.open(path,'rt') if path.suffix=='.gz' else path.open()
