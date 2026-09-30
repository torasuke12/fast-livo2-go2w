from pathlib import Path
import shutil, json,hashlib
import argparse
ap=argparse.ArgumentParser();ap.add_argument('capture',type=Path);ap.add_argument('work',type=Path);args=ap.parse_args()
p=args.capture;out=args.work/'data';run,=list((p/'diagnostics').glob('run-*'))
for folder in [out,out/'logs',out/'diagnostics']:
 folder.mkdir(parents=True,exist_ok=True);folder.chmod(0o755)
for item in p.iterdir():
 if item.is_file():shutil.copyfile(item,out/item.name)
for item in (p/'logs').iterdir():
 if item.is_file():shutil.copyfile(item,out/'logs'/item.name)
for item in run.iterdir():
 if item.name not in ['geometry.csv','correspondences.csv']:shutil.copyfile(item,out/'diagnostics'/item.name)
checks={}
for rel,expected in json.loads((p/'provenance/source-sha256.json').read_text()).items():
 file=Path.cwd()/rel;checks[rel]=(hashlib.sha256(file.read_bytes()).hexdigest()==expected)
(out/'source-identity.json').write_text(json.dumps(checks,indent=2)+'\n')
print('source matches',sum(checks.values()),'of',len(checks));print('differences',[k for k,v in checks.items() if not v])
print('run',run)
print('logs',[(x.name,x.stat().st_size) for x in (p/'logs').iterdir()])
