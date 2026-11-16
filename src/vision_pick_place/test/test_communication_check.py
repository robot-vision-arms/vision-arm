import importlib.util
import json
from pathlib import Path

from vision_pick_place.communication_check import CommunicationCheck, percentile95


def fill(check, seq=1, latency=10.):
    packet = dict(seq=seq, calib_ok=False, objects=[])
    check.receive('packet', packet)
    check.receive('target', packet.copy())
    check.receive('log', dict(seq=seq, acked=True, error='', latency_ms=latency))
    check.image_received = True


def test_tcp_failure_cannot_be_masked_by_later_successes():
    check = CommunicationCheck(5)
    check.receive('log', dict(seq=0, acked=False, error='connection refused'))
    for seq in range(1, 6):
        fill(check, seq)
    result = check.report()
    assert result['identical_packets'] == 5
    assert result['ok'] is False
    assert result['errors']


def test_100_successes_compute_independent_rates_and_p95():
    check = CommunicationCheck()
    for seq in range(100):
        fill(check, seq, seq)
    result = check.report()
    assert result['ok']
    assert result['ack_success_rate'] == result['target_delivery_rate'] == 1
    assert result['latency_p95_ms'] == 94.05
    assert result['latency_samples'] == 100


def test_ack_without_target_is_not_end_to_end_success():
    check = CommunicationCheck(1)
    fill(check)
    check.targets.clear()
    result = check.report()
    assert result['acked_packets'] == 1
    assert result['identical_packets'] == 0
    assert not result['ok']


def test_changed_coordinate_invalidates_delivery():
    check = CommunicationCheck(1)
    fill(check)
    check.targets[1]['objects'] = [{'x_mm': 999}]
    assert not check.report()['ok']


def test_duplicate_seq_and_invalid_latency_are_failures():
    check = CommunicationCheck(1)
    fill(check, latency=-10)
    check.receive('packet', check.packets[1])
    assert len(check.report()['errors']) == 2
    assert not check.report()['ok']


def test_missing_image_fails_unless_transport_only():
    check = CommunicationCheck(1)
    fill(check)
    check.image_received = False
    assert not check.report()['ok']
    check.require_image = False
    assert check.report()['ok']


def test_failure_report_replaces_prior_pass(tmp_path):
    root = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location('check_team_integration', root/'scripts/check_team_integration.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    out = tmp_path/'check.json'
    module.save_report(out, {'ok': True})
    module.save_report(out, {'ok': False, 'status': 'failed', 'errors': ['timeout']})
    assert json.loads(out.read_text())['ok'] is False
    assert list(tmp_path.iterdir()) == [out]
