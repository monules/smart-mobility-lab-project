"""Plain-language task requests for the Hub Control Center (pure Python).

Turns a sentence such as "run the robot escort with 3 trials, then S9" into
a plan: an ordered list of catalog tasks with validated options and a short
reason for each choice. It runs offline (keyword rules, no model, no network)
and never starts anything itself; the UI shows the plan and the user
confirms it.

Rules, in order:
  1. the request is split into steps at "then", ";" and "after that";
  2. an explicit id wins: S1-S9, T01-T18 (mapped to the scenario that checks
     them), "S9x"; otherwise each task scores one point per keyword found,
     plus one for its owner's name;
  3. numbers and option words are read only for the chosen task ("seed 4",
     "5 trials", "reactive", "hog", "without stabilisation", "for 90 s").
"""
import re

from smart_hub.control_center.catalog import BY_ID, resolve_options, TASKS, UNIT_SUITES

TEST_TO_TASK = {'T01': 'S1', 'T02': 'S1', 'T03': 'T03', 'T04': 'T04', 'T05': 'S3',
                'T06': 'S3', 'T07': 'S4', 'T08': 'S4', 'T09': 'S5', 'T10': 'S5',
                'T11': 'S6', 'T12': 'S6', 'T13': 'S7', 'T14': 'S7', 'T15': 'S8',
                'T16': 'S9', 'T17': 'S9x', 'T18': 'T18'}
OWNER_WORDS = {'husan': 'S1', 'tuymurod': 'S2', 'fazliddin': 'S3', 'arkar': 'S8',
               'carla': 'S4', 'aidil': 'S7', 'saidumar': 'S9'}
SUITE_WORDS = [('perception', 'perception (M1-A)'), ('husan', 'perception (M1-A)'),
               ('detect', 'perception (M1-A)'), ('demand', 'demand (M1-B)'),
               ('tuymurod', 'demand (M1-B)'), ('bus', 'bus (M2-A)'),
               ('fazliddin', 'bus (M2-A)'), ('coordinator', 'coordinator (M2-B)'),
               ('arkar', 'coordinator (M2-B)'), ('parking', 'parking and traffic (M3)'),
               ('traffic', 'parking and traffic (M3)'), ('carla', 'parking and traffic (M3)'),
               ('rout', 'routing and robot (M4)'), ('robot', 'routing and robot (M4)'),
               ('aidil', 'routing and robot (M4)'), ('safety', 'safety and simulator (M5)'),
               ('simulat', 'safety and simulator (M5)'), ('saidumar', 'safety and simulator (M5)'),
               ('control center', 'control center (M5)'), ('ui', 'control center (M5)')]
HELP_WORDS = ('help', 'what can you', 'what can i', 'list', 'options', 'menu')
STEP_SPLIT = re.compile(r'\bthen\b|;|\bafter that\b|&&|\band also\b', re.I)


def _explicit_task(text):
    """Return (task id, reason) for an explicit S/T id in text, else (None, '')."""
    m = re.search(r'\b[sS]9\s*x\b|\bs9[- ]?repeat', text)
    if m:
        return 'S9x', 'you named S9x'
    m = re.search(r'\bt\s?-?(0[1-9]|1[0-8])\b', text, re.I)
    if m:
        test = 'T' + m.group(1)
        return TEST_TO_TASK[test], '%s is checked by %s' % (test, TEST_TO_TASK[test])
    m = re.search(r'\bs\s?-?([1-9])\b', text, re.I)
    if m:
        sid = 'S' + m.group(1)
        if sid == 'S1' and re.search(r'real|lab|yolo', text):
            return 'S1-real', 'you named S1 with the real lab video'
        return sid, 'you named %s' % sid
    return None, ''


def _keyword_scores(text):
    words = set(re.findall(r'[a-z0-9_\-]+', text))
    words |= {w[:-1] for w in words if len(w) > 3 and w.endswith('s')}   # plurals
    scores = {}
    for task in TASKS:
        hits = [k for k in task['keywords'] if k in words or (' ' in k and k in text)]
        hits += [w for w, tid in OWNER_WORDS.items() if tid == task['id'] and w in words]
        if hits:
            scores[task['id']] = hits
    return scores


def _pick_task(text):
    """Return (task id or None, confidence, reason, alternatives)."""
    tid, reason = _explicit_task(text)
    if tid:
        return tid, 'high', reason, []
    scores = _keyword_scores(text)
    if not scores:
        return None, 'none', '', []
    ranked = sorted(scores, key=lambda t: (-len(scores[t]), TASKS.index(BY_ID[t])))
    best = ranked[0]
    tied = [t for t in ranked[1:] if len(scores[t]) == len(scores[best])]
    confidence = 'medium' if not tied and len(scores[best]) >= 2 else 'low'
    reason = 'matched: ' + ', '.join(scores[best])
    return best, confidence, reason, ranked[1:4]


def _number_after(text, pattern):
    m = re.search(pattern, text)
    return int(m.group(1)) if m else None


def _read_options(task, text):
    """Option values mentioned in text for task (unvalidated), plus notes."""
    names = {o['name'] for o in task['options']}
    found = {}
    seed = _number_after(text, r'seed\s*[=:#]?\s*(\d+)')
    if seed is not None and 'seed' in names:
        found['seed'] = seed
    trials = _number_after(text, r'(\d+)\s*(?:x\s*)?(?:trials?|runs?|times|dockings?|escorts?)')
    if trials is None:
        trials = _number_after(text, r'(?:trials?|runs?)\s*[=:]?\s*(\d+)')
    if trials is not None and 'nominal_trials' in names:
        found['nominal_trials'] = trials
    if 'dispatch' in names:
        for mode in ('reactive', 'predictive'):
            if mode in text:
                found['dispatch'] = mode
    if 'detector' in names:
        for det in ('yolo', 'hog', 'blob'):
            if re.search(r'\b%s\b' % det, text):
                found['detector'] = det
    if 'stabilize' in names and re.search(
            r'(without|no|not|un)[- ]?stabili[sz]', text):
        found['stabilize'] = False
    if 'target' in names and re.search(r'queue', text):
        found['target'] = 'queue'
    episodes = _number_after(text, r'(\d+)\s*episodes?')
    if episodes is not None and 'episodes' in names:
        found['episodes'] = episodes
    if task['run_for']:
        secs = _number_after(text, r'(\d+)\s*(?:s\b|sec|second)')
        mins = _number_after(text, r'(\d+)\s*(?:min|minute)')
        if secs is not None or mins is not None:
            found[task['run_for']] = secs if secs is not None else 60 * mins
    if 'suite' in names:
        for word, suite in SUITE_WORDS:
            if word in text:
                found['suite'] = suite
                break
    return found


def _validated(task, found):
    """Validate found options one by one; a bad value falls back to its default."""
    notes, ok = [], {}
    for name, value in found.items():
        try:
            resolve_options(task, {name: value})
            ok[name] = value
        except ValueError as exc:
            notes.append('%s, so the default is used' % exc)
    return resolve_options(task, ok), notes


def parse(request):
    """Return the plan for a plain-language request (see module docstring)."""
    text = ' '.join(str(request or '').lower().split())
    if not text:
        return {'plan': [], 'unmatched': [], 'help': True,
                'message': 'Type what you want to run, e.g. "S8 reactive seed 3".'}
    if any(w in text for w in HELP_WORDS) and not _explicit_task(text)[0]:
        return {'plan': [], 'unmatched': [], 'help': True,
                'message': 'I can run %d tasks: %s.' % (
                    len(TASKS), ', '.join('%s (%s)' % (t['id'], t['title']) for t in TASKS))}
    plan, unmatched = [], []
    for step in [s.strip(' ,.') for s in STEP_SPLIT.split(text)]:
        if not step:
            continue
        tid, confidence, reason, alternatives = _pick_task(step)
        if tid is None:
            unmatched.append(step)
            continue
        task = BY_ID[tid]
        options, notes = _validated(task, _read_options(task, step))
        plan.append({'task': tid, 'title': task['title'], 'owner': task['owner'],
                     'options': options, 'confidence': confidence, 'reason': reason,
                     'notes': notes, 'alternatives': alternatives, 'text': step})
    if plan:
        message = 'Plan: ' + ' → '.join(p['task'] for p in plan)
        if unmatched:
            message += ' (could not place: %s)' % '; '.join(unmatched)
    else:
        message = 'No task matched. Try a scenario id (S1-S9), a test id (T01-T18) ' \
                  'or words like "parking", "escort", "e-stop".'
    return {'plan': plan, 'unmatched': unmatched, 'help': False, 'message': message}


def suites():
    """Return the unit-test suite names (for the UI's help text)."""
    return list(UNIT_SUITES)
