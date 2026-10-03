import pytest

from ssb_tools.evaluate_ring_phase import phase_summary, ring_windows, run


def test_phase_labels_include_rejected_windows_and_preserve_failure_subchecks():
    windows=[dict(id=0,shape=[512,101],x_first_m=1.15,status='accepted'),
        dict(id=1,shape=[512,101],x_first_m=2.35,status='unmeasurable',reason='support',
             support_checks=dict(holdout_count=True,holdout_p95=False)),
        dict(id=2,shape=[512,101],x_first_m=1.5,status='accepted'),
        dict(id=3,status='unmeasurable',reason='no exposure')]
    result=phase_summary(windows,.001,1.2)
    assert result['ring_crossing']==2 and result['fraction_of_all_windows']==.5
    assert result['groups']['ring_crossing']['accepted']==1
    assert result['groups']['ring_crossing']['failed_support_checks']==dict(holdout_p95=1)
    assert result['eligible']==3


def test_noninteger_ring_pitch_ratio_still_has_repeating_phase():
    windows=[dict(id=i,shape=[512,11],x_first_m=.6*i-.005,status='accepted') for i in range(11)]
    assert ring_windows(windows,.001,1.5)=={0,5,10}


def test_phase_evaluation_cannot_write_into_production_directory(tmp_path):
    with pytest.raises(ValueError,match='evaluation/'):
        run(tmp_path/'absent',tmp_path/'public')
