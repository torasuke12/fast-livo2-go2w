"""Read-only extraction of a declared temporal sample from the large original CSVs."""
from pathlib import Path
import argparse, json, sys, time
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'humble_ws/src/fast-livo2/scripts'))
from analyze_lio_diagnostics import Table
ap=argparse.ArgumentParser();ap.add_argument('capture',type=Path);ap.add_argument('work',type=Path);a=ap.parse_args()
source,=list((a.capture/'diagnostics').glob('run-*'))
small=a.work/'data/diagnostics';out=a.work/'sample';out.mkdir(exist_ok=True)
f=Table(small/'geometry_states.csv');t=f['time']
targets=np.r_[np.arange(5,335.1,5),np.arange(170,220.1,1),np.arange(295,330.1,1),np.arange(320,323.01,.1)]
chosen=np.unique([np.argmin(abs(t-s)) for s in targets]);ids=set(map(int,f['scan_id'][chosen]));last_id=max(ids)
for name in ['scans.csv','iterations.csv','geometry_states.csv']:
 with (small/name).open() as src,(out/name).open('w') as dst:
  dst.write(next(src))
  for line in src:
   if int(line.split(',',1)[0]) in ids:dst.write(line)
(out/'metadata.json').write_text((small/'metadata.json').read_text())
(out/'selection.json').write_text(json.dumps({'source':str(source),'sampling':'nearest scan to 5s grid; 1s grid at 170-220 and 295-330; 0.1s grid at 320-323',
    'scan_ids':sorted(ids),'times':t[chosen].tolist(),'scans':len(ids)},indent=2)+'\n')
for name in ['geometry.csv','correspondences.csv']:
 start=time.monotonic();rows=0;kept=0
 with (source/name).open() as src,(out/name).open('w') as dst:
  dst.write(next(src))
  for line in src:
   scan=int(line.split(',',1)[0])
   if scan>last_id:break
   rows+=1
   if scan in ids:dst.write(line);kept+=1
   if rows%1000000==0:print(name,rows,'read',kept,'saved',round(time.monotonic()-start,1),'sec',flush=True)
 print(name,'complete',kept,'saved',round(time.monotonic()-start,1),'sec',flush=True)
out.chmod(0o755)
print('Selected',len(ids),'scans',flush=True)
