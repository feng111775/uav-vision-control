import json

from deploy_openmv import FORMAL_FILES, copy_verified, sha256
from validate_openmv_live import Metrics


def test_copy_verified_dry_run_does_not_write(tmp_path):
    source = tmp_path / "source"
    target = tmp_path / "target"
    source.mkdir()
    target.mkdir()
    for name in FORMAL_FILES:
        (source / name).write_text(name, encoding="utf-8")
    hashes = copy_verified(source, target, dry_run=True)
    assert set(hashes) == set(FORMAL_FILES)
    assert not list(target.iterdir())
    assert hashes["main.py"] == sha256(source / "main.py")


def test_metrics_parse_seven_fields_and_reject_bad_protocol():
    metrics = Metrics()
    metrics.line("D_TARGET,0,0,0,0,0,0,0")
    metrics.line("D_STATUS,CROSS_INVALID")
    metrics.line("D_TARGET,broken")
    report = metrics.report("test")
    assert report["d_target_count"] == 1
    assert report["valid_0"] == 1
    assert report["valid_1"] == 0
    assert report["status"] == {"CROSS_INVALID": 1}
    assert report["protocol_errors"] == 1
    json.dumps(report)
