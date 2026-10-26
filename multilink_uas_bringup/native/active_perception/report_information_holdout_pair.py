#!/usr/bin/env python3
"""Run unchanged per-stream checks only for acquired front and active streams."""
import argparse,json
from pathlib import Path
import report_view_array as report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--trial',type=Path,required=True);a=p.parse_args()
    report.ORDER=['front','active'];report.COLORS=['#2474b5','#111111']
    result=report.trial(a.trial)
    result['meaning']='Independent front and active monocular estimators, same flight. Four auxiliary estimators not started; six physical cameras remain rendered. All original stream/configuration checks retained for front and active.'
    (a.trial/'array_comparison.json').write_text(json.dumps(result,indent=2)+'\n')
