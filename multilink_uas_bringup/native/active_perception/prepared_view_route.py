#!/usr/bin/env python3
from pathlib import Path
from view_preparation import route_source
path=Path(__file__).with_name('unipilot_route.py')
source=route_source(path.read_text())
exec(compile(source,str(path),'exec'),dict(__name__='__main__',__file__=str(path)))
