"""Ordered shutdown, scoped strictly to process groups created by the demo runner."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def group_processes(group):
    found=[]
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():continue
        try:
            pid=int(path.name)
            if os.getpgid(pid)!=group or (path/'stat').read_text().split()[2]=='Z':continue
            args=(path/'cmdline').read_bytes().split(b'\0')
            found.append((pid,[v.decode(errors='replace') for v in args if v]))
        except (OSError,ProcessLookupError):pass
    return found


def wait_until(done, timeout):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if done():return True
        time.sleep(.1)
    return done()


def shutdown_children(children, session_path):
    report={'steps':[],'forced_groups':[]}
    def stop_child(child):
        if child.poll() is None:
            # launch는 자기 자식에게 신호를 전달하므로 부모에게만 한 번 보낸다.
            try:
                if child.args[:2]==['ros2','launch']:os.kill(child.pid,signal.SIGINT)
                else:os.killpg(child.pid,signal.SIGINT)
            except ProcessLookupError:pass
            try:child.wait(timeout=20)
            except subprocess.TimeoutExpired:return
        report['steps'].append({'label':getattr(child,'team2_label','child'),'returncode':child.poll()})
    sim=next((c for c in children if getattr(c,'team2_label','')=='sim'),None)
    # 새 명령을 보내는 쪽부터 끝내고, 서비스 제공자는 나중에 정리한다.
    for label in ['gui','vision']:
        for child in children:
            if getattr(child,'team2_label','')==label:stop_child(child)
    if sim is not None:
        # 통계 스레드가 끝날 때까지 /clock 브리지를 유지하고 서버 다음에 브리지를 끈다.
        for role in ['team2_pick_place_node','team2_move_group','move_group','team2_gz_server','gzserver','parameter_bridge']:
            for pid,args in group_processes(sim.pid):
                if not any(Path(arg).name==role for arg in args):continue
                try:os.kill(pid,signal.SIGINT)
                except ProcessLookupError:continue
                exited=wait_until(lambda:pid not in [p for p,_ in group_processes(sim.pid)],15)
                report['steps'].append({'role':role,'pid':pid,'exited':exited})
            if role=='team2_pick_place_node':
                # 이동 노드가 정리될 때까지 vacuum/스폰 서비스 제공자를 유지한다.
                for child in children:
                    if getattr(child,'team2_label','')=='link':stop_child(child)
        # Gazebo를 먼저 SIGINT로 끊지 않는다. 공식 서버 제어 서비스로 run loop를 끝낸다.
        # partition을 명시해 다른 사용자의 Gazebo 서버를 종료하지 않는다.
        def gazebo_gone():
            return not any('gz sim' in ' '.join(a) or 'gz-sim' in ' '.join(a) or
                           any(Path(arg).name in ('gzserver','team2_gz_server') for arg in a)
                           for p,a in group_processes(sim.pid) if p!=sim.pid)
        if sim.poll() is None and not gazebo_gone() and Path(session_path).exists():
            data=json.loads(Path(session_path).read_text());env=dict(os.environ,GZ_PARTITION=data['gz_partition'])
            try:
                result=subprocess.run(['gz','service','-s','/server_control','--reqtype','gz.msgs.ServerControl',
                    '--reptype','gz.msgs.Boolean','--timeout','3000','--req','stop: true'],
                    capture_output=True,text=True,timeout=5,env=env)
                report['server_stop_reply']=result.stdout.strip()
            except subprocess.TimeoutExpired:report['server_stop_reply']='timeout'
            report['gazebo_service_exit']=wait_until(gazebo_gone,15)
        report['gazebo_processes_exited']=gazebo_gone()
        stop_child(sim)
    else:
        for child in children:
            if getattr(child,'team2_label','')=='link':stop_child(child)
    for child in reversed(children):
        if child is not sim and getattr(child,'team2_label','') not in ['gui','vision','link']:stop_child(child)
    # 이 실행기가 만든 그룹에 살아 있는 프로세스가 있을 때만 강제 단계로 간다.
    # 강제 정리는 성공으로 숨기지 않고 결과에 남긴다.
    for child in children:
        if group_processes(child.pid):
            report['forced_groups'].append(child.pid)
            try:os.killpg(child.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            if not wait_until(lambda:not group_processes(child.pid),5):
                try:os.killpg(child.pid,signal.SIGKILL)
                except ProcessLookupError:pass
        try:child.wait(timeout=2)
        except subprocess.TimeoutExpired:pass
    report['remaining_processes']=[p for c in children for p in group_processes(c.pid)]
    return report
