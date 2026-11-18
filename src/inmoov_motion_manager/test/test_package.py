def test_package_has_expected_files():
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    assert (root / 'config' / 'gestures.yaml').exists()
    assert (root / 'inmoov_motion_manager' / 'motion_manager.py').exists()
