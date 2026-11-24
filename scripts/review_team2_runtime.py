"""Summarize Team 2 run logs without hiding recovered planning/shutdown errors."""
import argparse
import json
from pathlib import Path
import re


def review(directory):
    root=Path(directory)
    sim=(root/'sim.log').read_text(errors='replace') if (root/'sim.log').exists() else ''
    link=(root/'link.log').read_text(errors='replace') if (root/'link.log').exists() else ''
    process_logs=sim+'\n'+link
    # 계획 거부는 충돌 검사가 막은 사건이다. 최종 사이클 실패와 같은 뜻으로 합치지 않는다.
    counts={
        'self_collision_contacts':sum('constitutes a collision' in line and
            line.count("(type 'Robot link')")==2 for line in sim.splitlines()),
        'invalid_motion_plans':sim.count("failed with error code INVALID_MOTION_PLAN"),
        'acceleration_default_warnings':sim.count('Invalid max_acceleration_scaling_factor'),
        'octomap_unconfigured':sim.count('No 3D sensor plugin'),
        'absent_model_removals':len(re.findall(r'Entity named.*not found, so not removed',sim)),
        'native_shutdown_crashes':len(re.findall(r'process has died.*exit code -(?:11|6)\b',process_logs)),
        'abnormal_process_exits':len(re.findall(r'process has died.*exit code (?:-[0-9]+|[1-9][0-9]*)',process_logs)),
        'destroyable_shutdown_messages':sim.count('cannot use Destroyable because destruction was requested'),
        'statistics_shutdown_errors':sim.count("Exception in publisher thread: context cannot be slept with because it's invalid!"),
        'shutdown_escalations':len(re.findall(r'failed to terminate.*SIGINT',process_logs)),
        'gazebo_shutdown_escalations':sum('failed to terminate' in line and
            any(role in line for role in ['[gazebo-','[gzserver-','[team2_gz_server-']) for line in sim.splitlines()),
    }
    traces={}
    for name in ['sim','link','vision','gui']:
        p=root/(name+'.log')
        text=p.read_text(errors='replace') if p.exists() else ''
        traces[name]={'tracebacks':text.count('Traceback (most recent call last)'),
                      'keyboard_interrupts':text.count('KeyboardInterrupt')}
    counts.update(verified_request_recoveries=link.count('TEAM2_GZ_REQUEST_RECOVERED'),
                  unconfirmed_requests=link.count('TEAM2_GZ_REQUEST_UNCONFIRMED'),
                  injected_reply_losses=link.count('TEAM2_TEST_REPLY_DROPPED'))
    return {'counts':counts,'python_exceptions':traces,
            'note':'Counts are log events, not failed pick counts. Zero observed errors is scoped to this run.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory');parser.add_argument('--out')
    args=parser.parse_args();result=review(args.directory)
    text=json.dumps(result,ensure_ascii=False,indent=2)
    if args.out:Path(args.out).write_text(text+'\n')
    print(text)


if __name__=='__main__':main()
