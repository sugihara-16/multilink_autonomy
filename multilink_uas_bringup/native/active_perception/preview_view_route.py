#!/usr/bin/env python3
from pathlib import Path
from preview_preparation import route_source
path=Path(__file__).with_name('unipilot_route.py')
exec(compile(route_source(path.read_text()),str(path),'exec'),dict(__name__='__main__',__file__=str(path)))
