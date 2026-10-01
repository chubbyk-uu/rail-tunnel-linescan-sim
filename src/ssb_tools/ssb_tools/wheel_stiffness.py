"""Calibrate the sprung-wheel joint stiffness against DART's realized static deflection.

With a 1 ms step, DART's implicit joint spring/damper combined with wheel contact constraints
does not settle at load/k: measured deflections are 1.3-2x larger and include a roughly constant
numerical part (about 0.08 mm here). The physical assumption is the realized static deflection of
the polyurethane tread, so the SDF stiffness is solved for it on an isolated vehicle with the same
sprung/unsprung masses, wheel radius, damping ratio and step. Requires the gz-sim Python bindings.
"""
import math
import tempfile
from pathlib import Path

GRAVITY = 9.81
SETTLE_STEPS = 2000


def _sdf(k, c, sprung, axle, wheel, radius, step, inertia):
    wheels = ''
    for i, (x, y) in enumerate(((-.35, .75), (.35, .75), (-.35, -.75), (.35, -.75))):
        wheels += f'''<link name="axle{i}"><pose>{x} {y} {radius} 0 0 0</pose><inertial><mass>{axle}</mass>
<inertia><ixx>5e-4</ixx><iyy>5e-4</iyy><izz>5e-4</izz></inertia></inertial></link>
<joint name="s{i}" type="prismatic"><parent>base</parent><child>axle{i}</child><axis><xyz>0 0 1</xyz>
<limit><lower>-0.005</lower><upper>0.005</upper></limit><dynamics><spring_reference>0</spring_reference>
<spring_stiffness>{k}</spring_stiffness><damping>{c}</damping></dynamics></axis></joint>
<link name="w{i}"><pose>{x} {y} {radius} 0 0 0</pose><inertial><mass>{wheel}</mass>
<inertia><ixx>.013</ixx><iyy>.025</iyy><izz>.013</izz></inertia></inertial>
<collision name="c"><pose>0 0 0 1.5707963 0 0</pose><geometry><cylinder><radius>{radius}</radius>
<length>0.06</length></cylinder></geometry></collision></link>
<joint name="r{i}" type="revolute"><parent>axle{i}</parent><child>w{i}</child><axis><xyz>0 1 0</xyz>
<limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>'''
    ixx, iyy, izz = inertia
    return f'''<?xml version="1.0"?><sdf version="1.9"><world name="calibration">
<physics name="step" type="dart"><max_step_size>{step}</max_step_size><real_time_factor>0</real_time_factor></physics>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/><gravity>0 0 -{GRAVITY}</gravity>
<model name="ground"><static>true</static><link name="g"><collision name="c"><pose>0 0 -0.05 0 0 0</pose>
<geometry><box><size>10 3 0.1</size></box></geometry></collision></link></model>
<model name="car"><link name="base"><pose>0 0 0.3 0 0 0</pose><inertial><mass>{sprung}</mass>
<inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz></inertia></inertial></link>{wheels}</model></world></sdf>'''


def realized_deflection(k, zeta, sprung, axle, wheel, radius, step, inertia=(4.2, 8.1, 10.5)):
    """Mean settled spring compression (m) and the SDF damping used, from one isolated run."""
    from gz.common5 import set_verbosity
    from gz.sim8 import Joint, Model, TestFixture, World
    set_verbosity(1)
    c = 2*zeta*math.sqrt(k*sprung/4)
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder)/'calibration.sdf'
        path.write_text(_sdf(k, c, sprung, axle, wheel, radius, step, inertia))
        fixture = TestFixture(str(path)); joints = []; samples = []

        def post(info, ecm):
            if not joints:
                model = Model(World(1).model_by_name(ecm, 'car'))
                for i in range(4):
                    joint = Joint(model.joint_by_name(ecm, f's{i}')); joint.enable_position_check(ecm, True)
                    joints.append(joint)
                return
            samples.append(sum(j.position(ecm)[0] for j in joints)/4)
        fixture.on_post_update(post); fixture.finalize()
        fixture.server().run(True, SETTLE_STEPS, False)
    tail = samples[-200:]
    if max(tail)-min(tail) > 1e-6:
        raise RuntimeError('calibration vehicle did not settle')
    return sum(tail)/len(tail), c


def calibrate(target, zeta, sprung, axle, wheel, radius, step, tolerance=.002):
    """SDF stiffness/damping whose realized static deflection equals target (relative tolerance)."""
    nominal = sprung*GRAVITY/4/target
    # Measured form: deflection ~ a/k + b. Secant iteration on 1/k from the nominal guess.
    history = []
    k = nominal
    for _ in range(8):
        q, c = realized_deflection(k, zeta, sprung, axle, wheel, radius, step)
        history.append(dict(stiffness_n_m=k, damping_n_s_m=c, deflection_m=q))
        if abs(q-target) <= tolerance*target:
            return dict(stiffness_n_m=k, damping_n_s_m=c, realized_static_deflection_m=q,
                        nominal_stiffness_n_m=nominal, iterations=history,
                        method='isolated DART vehicle, same masses/radius/step; mean of four springs')
        if len(history) == 1:
            k = k*q/target  # first step: scale as if linear
        else:
            a, b = history[-2], history[-1]
            slope = (b['deflection_m']-a['deflection_m'])/(1/b['stiffness_n_m']-1/a['stiffness_n_m'])
            inverse = 1/b['stiffness_n_m']+(target-b['deflection_m'])/slope
            if inverse <= 0:
                raise ValueError(f'target deflection {target*1e3:.3f} mm is below what DART realizes at this step')
            k = 1/inverse
    raise RuntimeError('wheel stiffness calibration did not converge')
