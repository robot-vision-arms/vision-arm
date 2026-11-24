"""Create ten-slot workcell assets without editing other teammates' model files."""
import argparse
import json
import os
from pathlib import Path
from ur5e_pick_place.team2_demo_profile import generate_profile

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',default='logs/team2_demo_profile')
    p.add_argument('--realistic',action='store_true');p.add_argument('--randomize',action='store_true')
    p.add_argument('--seed',type=int,default=0);p.add_argument('--speed',type=float,default=.5)
    p.add_argument('--screw-length-mm',type=float,default=os.environ.get('TEAM2_SCREW_LENGTH_MM','40'),
                   help='Fixed simulation length, 10..40 mm; default TEAM2_SCREW_LENGTH_MM or 40')
    a=p.parse_args()
    print(json.dumps(generate_profile(a.out,Path(__file__).resolve().parents[1],realistic=a.realistic,randomize=a.randomize,seed=a.seed,speed=a.speed,screw_length_mm=a.screw_length_mm),indent=2))
