"""Independent straight-track contact/encoder/scan checks from archived diagnostics."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml


def validate(root,config):
    root=Path(root);c=yaml.safe_load(Path(config).read_text())
    a=np.genfromtxt(root/'evaluation/contact.csv',delimiter=',',names=True);a=a[a['t']>=0]
    b=np.genfromtxt(root/'metadata/encoders.csv',delimiter=',',names=True)
    summary=json.loads((root/'summary.json').read_text())
    count=c['odometer']['ppr']*c['odometer']['edges_per_cycle']*c['odometer']['gear_ratio']
    dl=c['calibration'].get('odo_left_diameter_m',c['calibration']['wheel_diameter_m'])
    dr=c['calibration'].get('odo_right_diameter_m',c['calibration']['wheel_diameter_m'])
    tl=c['truth'].get('odo_left_diameter_m',c['truth']['wheel_diameter_m'])
    tr=c['truth'].get('odo_right_diameter_m',c['truth']['wheel_diameter_m'])
    estimated=np.pi/count*(b['count_left']*dl+b['count_right']*dr)/2
    target=np.deg2rad(c['motion']['start_theta_deg'])+2*np.pi*estimated/c['motion']['advance_per_rev_m']
    travel=a['x'][-1]-a['x'][0];turns=(a['scan'][-1]-a['scan'][0])/(2*np.pi)
    pitch=travel/turns if turns>0 else None;expected=c['motion']['advance_per_rev_m']/((dl/tl+dr/tr)/2)
    slip=max(float(np.max(abs(a['vx']-a['left_rate']*tl/2))),float(np.max(abs(a['vx']-a['right_rate']*tr/2))))
    checks=dict(complete=bool(summary['complete']),positive_travel=bool(travel>1),
        no_reverse=bool(a['vx'].min()>-1e-4),lateral_guidance=bool(abs(a['y']).max()<.001),
        supported=bool(abs(a['z']-.3).max()<.003),attitude=bool(max(abs(a['roll']).max(),abs(a['pitch']).max())<.02),
        small_longitudinal_slip=slip<.001,
        encoder_distance=bool(np.max(abs(estimated-b['s_hat']))<1e-10),
        encoder_scan_target=bool(np.max(abs(target-b['theta_target']))<1e-9),
        scan_tracking=bool(np.max(abs(b['theta_target']-b['scan']))<.012),
        diameter_error_pitch=bool(pitch and abs(pitch/expected-1)<.001))
    report=dict(passed=all(checks.values()),checks=checks,travel_m=float(travel),estimated_m=float(estimated[-1]),
        measured_pitch_m=pitch,expected_pitch_m=expected,max_slip_speed_m_s=slip,
        max_lateral_m=float(abs(a['y']).max()),max_scan_error_rad=float(abs(b['theta_target']-b['scan']).max()),
        limitation='straight low-slip check; not a derailment or arbitrary-curve certification')
    (root/'validation.json').write_text(json.dumps(report,indent=2)+'\n');return report


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('diagnostics');p.add_argument('--config',required=True)
    a=p.parse_args();r=validate(a.diagnostics,a.config);print(json.dumps(r,indent=2));return 0 if r['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
