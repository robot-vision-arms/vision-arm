"""The shutdown helper must never signal a different process session."""
import importlib.util
from pathlib import Path
import subprocess


def test_runtime_review_includes_separate_rviz_process(tmp_path):
    path=Path(__file__).resolve().parents[3]/'scripts/review_team2_runtime.py'
    spec=importlib.util.spec_from_file_location('team2_runtime_review_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    (tmp_path/'rviz.log').write_text('[ERROR] [team2_rviz-1]: process has died [exit code -11]\n')
    result=module.review(tmp_path)
    assert result['counts']['abnormal_process_exits']==1
    assert result['counts']['native_shutdown_crashes']==1


def test_shutdown_leaves_other_session_running(tmp_path):
    path=Path(__file__).resolve().parents[3]/'scripts/team2_shutdown.py'
    spec=importlib.util.spec_from_file_location('team2_shutdown_test',path)
    helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
    owned=subprocess.Popen(['/bin/sleep','5'],start_new_session=True)
    unrelated=subprocess.Popen(['/bin/sleep','5'],start_new_session=True)
    owned.team2_label='test_child'
    try:
        assert owned.pid in [pid for pid,_ in helper.group_processes(owned.pid)]
        assert unrelated.pid not in [pid for pid,_ in helper.group_processes(owned.pid)]
        result=helper.shutdown_children([owned],tmp_path/'no_session.json')
        assert owned.poll() is not None
        assert unrelated.poll() is None
        assert result['forced_groups']==[] and result['remaining_processes']==[]
    finally:
        for child in [owned,unrelated]:
            if child.poll() is None:child.terminate()
            child.wait(timeout=3)
