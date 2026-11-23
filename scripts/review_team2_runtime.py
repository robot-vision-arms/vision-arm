"""Summarize Team 2 run logs without hiding recovered planning/shutdown errors."""
import argparse
import json
from pathlib import Path
import re


def review(directory):
    root=Path(directory)
    sim=(root/'sim.log').read_text(errors='replace') if (root/'sim.log').exists() else ''
    # 계획 거부는 충돌 검사가 막은 사건이다. 최종 사이클 실패와 같은 뜻으로 합치지 않는다.
    counts={
        'self_collision_contacts':sum('constitutes a collision' in line and
            line.count("(type 'Robot link')")==2 for line in sim.splitlines()),
        'invalid_motion_plans':sim.count("failed with error code INVALID_MOTION_PLAN"),
        'acceleration_default_warnings':sim.count('Invalid max_acceleration_scaling_factor'),
        'octomap_unconfigured':sim.count('No 3D sensor plugin'),
        'absent_model_removals':len(re.findall(r'Entity named.*not found, so not removed',sim)),
        'native_shutdown_crashes':len(re.findall(r'process has died.*exit code -11',sim)),
        'gazebo_shutdown_escalations':len(re.findall(r'failed to terminate.*SIGINT',sim)),
    }
    traces={}
    for name in ['sim','link','vision','gui']:
        p=root/(name+'.log')
        text=p.read_text(errors='replace') if p.exists() else ''
        traces[name]={'tracebacks':text.count('Traceback (most recent call last)'),
                      'keyboard_interrupts':text.count('KeyboardInterrupt')}
    return {'counts':counts,'python_exceptions':traces,
            'note':'Counts are log events, not failed pick counts. Native shutdown crashes remain unresolved.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory');parser.add_argument('--out')
    args=parser.parse_args();result=review(args.directory)
    text=json.dumps(result,ensure_ascii=False,indent=2)
    if args.out:Path(args.out).write_text(text+'\n')
    print(text)


if __name__=='__main__':main()
