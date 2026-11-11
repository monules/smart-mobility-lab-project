"""Task catalog of the Hub Control Center (M5, pure Python, no ROS).

Every runnable task of the assignment is one entry: scenario checks S1-S9,
the stand-alone tests (T03, T04, T18), the real-video evaluation and the
unit-test suites per subsystem. An entry says who owns it, which options a
user may choose, and how those options become one command line. The UI, the
intent parser and the run manager all read this table, so a task added here
appears everywhere.

Commands are argument lists (never a shell string): option values are
validated against the catalog first, so nothing typed in the UI reaches a
shell.
"""
import os

REAL_VIDEO = 'videos/lab/IMG_0572.MOV'
YOLO_WEIGHTS = 'models/yolov4-tiny/yolov4-tiny.weights'
LABELS = 'evidence/m1a/real_video/labels_husan.csv'
IMAGE_POINTS = '488 199 1224 207 1248 335 457 330 933 546 1118 550 1173 1018 943 1018'
MAP_POINTS = '2 7 6 7 6 5 2 5 4.5 4.75 5.5 4.75 5.5 1.25 4.5 1.25'

UNIT_SUITES = {
    'perception (M1-A)': ['test_m1a_units.py', 'test_m1a_pipeline.py',
                          'test_m1a_video_tools.py', 'test_video_eval.py',
                          'test_camera_model.py'],
    'demand (M1-B)': ['test_demand_data.py', 'test_demand_features.py',
                      'test_demand_models.py', 'test_demand_estimator.py',
                      'test_demand_dispatch_policy.py'],
    'bus (M2-A)': ['test_bus_core.py'],
    'coordinator (M2-B)': ['test_coordinator_core.py'],
    'parking and traffic (M3)': ['test_t07_parking_atomicity.py', 'test_t08_parking_expiry.py',
                                 'test_t09_congestion.py', 'test_t10_stale_state.py'],
    'routing and robot (M4)': ['test_route_core.py', 'test_robot_core.py'],
    'safety and simulator (M5)': ['test_safety_core.py', 'test_bridge_core.py',
                                  'test_sim_core.py', 'test_geometry.py',
                                  'test_metrics_core.py', 'test_marker_core.py'],
    'control center (M5)': ['test_control_center.py'],
}


def _choice(name, label, choices, default, help_text=''):
    return {'name': name, 'label': label, 'type': 'choice', 'choices': list(choices),
            'default': default, 'help': help_text}


def _int(name, label, default, lo, hi, help_text=''):
    return {'name': name, 'label': label, 'type': 'int', 'default': default,
            'min': lo, 'max': hi, 'help': help_text}


def _bool(name, label, default, help_text=''):
    return {'name': name, 'label': label, 'type': 'bool', 'default': default,
            'help': help_text}


def _task(tid, title, owner, group, summary, minutes, keywords, options=(), **how):
    return dict({'id': tid, 'title': title, 'owner': owner, 'group': group,
                 'summary': summary, 'minutes': minutes, 'keywords': list(keywords),
                 'options': list(options), 'launch': None, 'check': None,
                 'command': None, 'run_for': None, 'requires': [], 'live': True}, **how)


TASKS = [
    _task('S1', 'Passenger detection', 'Husan · M1-A', 'Perception',
          'Synthetic camera: count people in queue Q and crossing C (T01, T02).',
          1, ['passenger', 'detect', 'detection', 'count', 'counting', 'camera', 'queue',
              'people', 'occlusion', 't01', 't02', 'blob', 'hog'],
          [_choice('detector', 'Detector', ['blob', 'hog'], 'blob',
                   'blob = baseline for the synthetic figures, hog = pretrained people detector'),
           _int('seed', 'Seed', 1, 1, 99)],
          launch='s1.launch.py', check='s1_check.json'),
    _task('S1-real', 'Real lab video evaluation', 'Husan · M1-A', 'Perception',
          'Score a detector on the 141 hand-labelled frames of the lab video.',
          4, ['real', 'video', 'lab', 'yolo', 'labelled', 'labeled', 'stabilise', 'stabilize',
              'stabilisation', 'stabilization', 'mae'],
          [_choice('detector', 'Detector', ['yolo', 'hog'], 'yolo'),
           _bool('stabilize', 'Stabilise the image', True,
                 'align every frame to the first one before counting')],
          command='video_eval', check='results.json', live=False,
          requires=[REAL_VIDEO, YOLO_WEIGHTS]),
    _task('S2', 'Demand prediction (live)', 'Tuymurod · M1-B', 'Prediction',
          'Replay a synthetic queue and watch the 30 s forecast against persistence.',
          2, ['demand', 'forecast', 'predict', 'prediction', 'ai', 'model', 'ridge', 's2'],
          [_int('seed', 'Seed', 1025, 1, 9999),
           _int('run_for', 'Run for (s)', 120, 70, 600,
                'the forecast becomes valid after the 60 s warm-up')],
          launch='demand_demo.launch.py', run_for='run_for'),
    _task('T03', 'Retrain and evaluate the demand model', 'Tuymurod · M1-B', 'Prediction',
          'Train on whole episodes, select on validation, report held-out MAE vs persistence.',
          3, ['train', 'retrain', 'training', 'evaluate', 'demand', 'model', 'held-out',
              'heldout', 'split', 't03',
              'persistence', 'baseline'],
          [_choice('target', 'Forecast target', ['demand', 'queue'], 'demand',
                   'demand = no-boarding demand (shipped), queue = future queue count'),
           _int('episodes', 'Episodes', 50, 30, 200, 'the assignment needs at least 30')],
          command='train', check='metrics.json', live=False),
    _task('S3', 'Bus docking', 'Fazliddin · M2-A', 'Bus',
          'Dock the bus at bay B through crossing C; cancel, pedestrian, blocked bay '
          'and stale-pose cases (T05, T06).',
          4, ['bus', 'dock', 'docking', 'bay', 'door', 't05', 't06'],
          [_int('nominal_trials', 'Nominal dockings', 3, 1, 10, 'the report used 10')],
          launch='s3.launch.py', check='s3_check.json'),
    _task('T04', 'Bus dispatch from the forecast', 'Arkar · M2-B', 'Coordination',
          'Coordinator dispatch: noise, one goal per mission, invalid forecast, '
          'feed gap, FALLBACK.',
          4, ['dispatch', 'coordinator', 'hysteresis', 'fallback', 't04'],
          launch='t04.launch.py', check='t04_check.json'),
    _task('S4', 'Parking reservations', 'Carla · M3', 'Parking and traffic',
          'Two cars ask for P2 at once, idempotent retry, wrong token, expiry (T07, T08).',
          1, ['parking', 'park', 'slot', 'reserve', 'reservation', 'token', 'barrier',
              't07', 't08'],
          [_int('seed', 'Seed', 4, 1, 99)],
          launch='s4.launch.py', check='s4_check.json'),
    _task('S5', 'Congestion detection', 'Carla · M3', 'Parking and traffic',
          'Label roads FREE / DENSE / BLOCKED / UNKNOWN from simulated traffic (T09, T10).',
          2, ['congestion', 'traffic', 'dense', 'jam', 'blocked', 'edge', 'stale', 't09', 't10'],
          [_int('seed', 'Seed', 5, 1, 99)],
          launch='s5.launch.py', check='s5_check.json'),
    _task('S6', 'Route replanning', 'Aidil · M4', 'Routing and robot',
          'Cheapest valid route, blocked road, NO_ROUTE with zero motion (T11, T12).',
          2, ['route', 'routing', 'replan', 'reroute', 'planner', 'path', 'dijkstra',
              'no_route', 't11', 't12'],
          launch='s6.launch.py', check='s6_check.json'),
    _task('S7', 'Robot escort', 'Aidil · M4', 'Routing and robot',
          'Escort a passenger R to D with obstacle detour, readiness timeout, cancel (T13, T14).',
          5, ['robot', 'escort', 'passenger', 'obstacle', 'detour', 'ready', 'readiness',
              't13', 't14'],
          [_int('nominal_trials', 'Nominal escorts', 2, 1, 10, 'the report used 10')],
          launch='s7.launch.py', check='s7_check.json'),
    _task('S8', 'Rush hour (everything together)', 'Arkar · M2-B + all', 'Integration',
          'One full rush-hour episode: detection, forecast, dispatch, docking, parking, '
          'escort, obstacle (T15).',
          3, ['rush', 'hour', 'integration', 'integrated', 'everything', 'together', 'full',
              'predictive', 'reactive', 't15', 'episode'],
          [_int('seed', 'Seed', 1, 1, 99),
           _choice('dispatch', 'Dispatch mode', ['predictive', 'reactive'], 'predictive')],
          launch='s8.launch.py', check='s8_episode.json'),
    _task('S9', 'Faults and emergency stop', 'Saidumar · M5', 'Safety',
          'E-stop latch, stale data, reset rules, no automatic restart (T16).',
          1, ['safety', 'fault', 'faults', 'estop', 'e-stop', 'emergency', 'stop', 'latch',
              'reset', 't16'],
          [_int('seed', 'Seed', 9, 1, 99)],
          launch='s9.launch.py', check='s9_check.json'),
    _task('S9x', 'Fault repetition (5 faults x 5 phases)', 'Saidumar · M5', 'Safety',
          'Every fault at start, cruise, turn, top speed and reverse; 25 trials.',
          4, ['repeat', 'repetition', 'phases', 'watchdog', 'every', 't17'],
          launch='s9_repeat.launch.py', check='s9_repeat.json'),
    _task('T18', 'Planner loss', 'Aidil · M4 + Saidumar · M5', 'Safety',
          'Planner absent or late: 1 s timeout x 3, then PLANNER_UNAVAILABLE, no motion.',
          1, ['planner', 'service', 'loss', 'lost', 'timeout', 'late', 'unavailable', 't18'],
          launch='t18.launch.py', check='t18_check.json'),
    _task('UNIT', 'Unit tests of one subsystem', 'owner of the subsystem', 'Tests',
          'Run the pure-Python unit tests of one subsystem (seconds, no ROS graph).',
          1, ['unit', 'tests', 'pytest', 'suite'],
          [_choice('suite', 'Subsystem', list(UNIT_SUITES), 'safety and simulator (M5)')],
          command='pytest', live=False),
]

BY_ID = {t['id']: t for t in TASKS}


def public_catalog(root):
    """Return the catalog for the UI, marking tasks whose input files are missing."""
    out = []
    for t in TASKS:
        missing = [p for p in t['requires'] if not os.path.exists(os.path.join(root, p))]
        out.append(dict({k: v for k, v in t.items() if k != 'requires'},
                        available=not missing, missing=missing))
    return out


def resolve_options(task, given):
    """Return the full, validated option dict for task (defaults filled in).

    Raises ValueError with a message the UI can show for an unknown option or
    a value outside the catalog's range.
    """
    given = dict(given or {})
    unknown = set(given) - {o['name'] for o in task['options']}
    if unknown:
        raise ValueError('unknown option(s) for %s: %s' % (task['id'], ', '.join(sorted(unknown))))
    values = {}
    for opt in task['options']:
        raw = given.get(opt['name'], opt['default'])
        if opt['type'] == 'int':
            try:
                value = int(raw)
            except (TypeError, ValueError):
                raise ValueError('%s must be a whole number' % opt['label'])
            if not opt['min'] <= value <= opt['max']:
                raise ValueError('%s must be between %d and %d'
                                 % (opt['label'], opt['min'], opt['max']))
        elif opt['type'] == 'bool':
            if isinstance(raw, str):
                raw = raw.strip().lower() in ('1', 'true', 'yes', 'on')
            value = bool(raw)
        else:
            value = str(raw)
            if value not in opt['choices']:
                raise ValueError('%s must be one of: %s'
                                 % (opt['label'], ', '.join(opt['choices'])))
        values[opt['name']] = value
    return values


def _launch_command(task, values, out_dir):
    args = ['ros2', 'launch', 'smart_hub', task['launch']]
    if task['id'] == 'S2':
        args.append('run_id:=ui-' + os.path.basename(out_dir))
    else:
        args.append('output_dir:=' + out_dir)
    for name, value in values.items():
        if name == task['run_for']:
            continue
        if isinstance(value, bool):
            value = 'true' if value else 'false'
        args.append('%s:=%s' % (name, value))
    return args


def build_command(task, values, out_dir):
    """Return the argument list that runs task with validated values into out_dir."""
    if task['launch']:
        return _launch_command(task, values, out_dir)
    if task['command'] == 'video_eval':
        args = ['ros2', 'run', 'smart_hub', 'video_eval', '--video', REAL_VIDEO,
                '--detector', values['detector'], '--labels', LABELS, '--out', out_dir,
                '--image-points'] + IMAGE_POINTS.split() + ['--map-points'] + MAP_POINTS.split()
        if values['stabilize']:
            args.append('--stabilize')
        if values['detector'] == 'hog':
            args += ['--hog-upscale', '0.5']
        return args
    if task['command'] == 'train':
        return ['python3', 'src/smart_hub/scripts/train_demand_model.py',
                '--target', values['target'], '--episodes', str(values['episodes']),
                '--out-dir', out_dir, '--model-out', os.path.join(out_dir, 'model.json'),
                '--no-plots']
    if task['command'] == 'pytest':
        files = [os.path.join('src/smart_hub/test', f) for f in UNIT_SUITES[values['suite']]]
        return ['python3', '-m', 'pytest', '-q', '-p', 'no:cacheprovider'] + files
    raise ValueError('task %s has no command' % task['id'])
