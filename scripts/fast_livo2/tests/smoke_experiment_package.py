#!/usr/bin/env python3
"""Explicit developer smoke test, not a campaign prefix or full capture.

Usage: python3 smoke_experiment_package.py PACKAGE NEW_OUTPUT
Runs 6 seconds of sensor time for baseline/current A/B/C on sep07, sequentially.
No historical capture is used; does not qualify a condition for full acquisition.
"""
import json
import argparse
import fcntl
import os
from pathlib import Path
import sys
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from experiment_suite import verify,host_ok
from experiment.supervisor import supervise

guard=open(f'/tmp/fast-livo2-experiment-{os.getuid()}.lock','a')
fcntl.flock(guard,fcntl.LOCK_EX|fcntl.LOCK_NB)
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('package',type=Path); parser.add_argument('output',type=Path)
parser.add_argument('--conditions',nargs='+',choices=[v+'_'+m for v in ('baseline','current') for m in ('A','B','C')])
args=parser.parse_args()
package=args.package.resolve(); out=args.output.resolve()
if out.exists(): raise SystemExit('Smoke output must be new')
manifest,lock=verify(package)
manifest['config']['prefix_seconds']=6
out.mkdir(parents=True)
(out/'validation_only.json').write_text(json.dumps(dict(package_id=lock['package_id'],seconds=6,qualifies_for_full=False),indent=2)+'\n')
for variant in ('baseline','current'):
    for mode in ('A','B','C'):
        if args.conditions and variant+'_'+mode not in args.conditions: continue
        host_ok(manifest['resources'])
        condition=dict(id='sep07_'+variant+'_'+mode,bag='sep07',variant=variant,mode=mode)
        result=supervise(package,manifest,condition,out/condition['id'],'prefix')
        print(condition['id'],result,flush=True)
        if result['status']=='failed': raise SystemExit(1)
