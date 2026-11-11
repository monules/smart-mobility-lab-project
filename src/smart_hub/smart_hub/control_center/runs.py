"""Run queue of the Hub Control Center (M5, pure Python, no ROS).

One task runs at a time: two launches would share the ROS graph and the
simulator, so their results would not be independent evidence. Further
requests wait in a queue (a plan "S4 then S5" becomes two queued runs).

Each run gets its own folder output/ui/<run_id>/ with the checker's JSON and
the full console log (log.txt). When the process ends, the verdict is read
from the checker's file, never from the exit code alone (ros2 launch exits 0
even when a check fails). Stop sends SIGINT to the whole process group (ROS
nodes shut down cleanly), then SIGTERM and SIGKILL if it does not end.
"""
import collections
import json
import os
import signal
import subprocess
import threading
import time

from smart_hub.control_center.catalog import build_command, BY_ID, resolve_options

LOG_LINES = 400        # lines kept in memory per run (the full log is on disk)
HISTORY_MAX = 200
STOP_GRACE = (8.0, 3.0)  # s after SIGINT before SIGTERM, then before SIGKILL


def _check_list(checks):
    return [{'check': c.get('check', '?'), 'passed': bool(c.get('passed')),
             'detail': str(c.get('detail', ''))} for c in checks or []]


def _verdict_real_video(r):
    checks = [
        {'check': 'count MAE ≤ 1 (assignment 8.3)', 'passed': bool(r.get('target_mae_le_1')),
         'detail': 'MAE %.3f over %s labelled frames' % (r.get('count_mae', -1),
                                                         r.get('labelled_frames'))},
        {'check': 'exact count rate', 'passed': True,
         'detail': '%.1f %%' % (100 * (r.get('exact_count_rate') or 0))},
        {'check': 'crossing flag accuracy', 'passed': True,
         'detail': '%.1f %%' % (100 * (r.get('crossing_accuracy') or 0))}]
    if r.get('frames_partial_occlusion'):
        checks.append({'check': 'partially occluded frames', 'passed': True,
                       'detail': 'MAE %.3f on %d frames' % (r['mae_partial_occlusion'],
                                                            r['frames_partial_occlusion'])})
    return bool(r.get('target_mae_le_1')), checks


def _verdict_training(r):
    target = r.get('shipped_target', '?')
    test = r.get('test', {}).get(target, {})
    checks = [{'check': name.replace('_', ' '), 'passed': bool(ok), 'detail': ''}
              for name, ok in sorted(r.get('pass_criteria', {}).items())]
    checks.insert(0, {'check': 'held-out MAE (%s target)' % target,
                      'passed': test.get('ridge_mae', 9) <= test.get('persistence_mae', 0),
                      'detail': 'ridge %.3f vs persistence %.3f' % (
                          test.get('ridge_mae', float('nan')),
                          test.get('persistence_mae', float('nan')))})
    return all(c['passed'] for c in checks), checks


def _verdict_repeat(r):
    checks = [{'check': fault.replace('_', ' '), 'passed': s['passed'] == s['trials'],
               'detail': '%d/%d, zero command max %.3f s (limit %.2f s)' % (
                   s['passed'], s['trials'], s['zero_latency_max_s'], s['latency_limit_s'])}
              for fault, s in r.get('summary', {}).items()]
    return bool(r.get('passed')), checks


def _verdict_episode(r):
    v = r.get('safety_violations', {})
    m = r.get('metrics', {})
    checks = [{'check': 'no %s violations' % k.replace('_', ' '), 'passed': n == 0,
               'detail': str(n)} for k, n in sorted(v.items())]
    checks.append({'check': 'bus mission', 'passed': m.get('bus_mission') == 'SUCCEEDED',
                   'detail': '%s, dispatched by %s at %s s' % (
                       m.get('bus_mission'), m.get('dispatch_source'), m.get('dispatch_t_s'))})
    checks.append({'check': 'robot escort', 'passed': m.get('robot_mission') == 'SUCCEEDED',
                   'detail': '; '.join(m.get('robot_reroutes', [])) or 'no reroute'})
    checks.append({'check': 'queue wait p50 / p95', 'passed': True,
                   'detail': '%.1f / %.1f s, %s boarded' % (
                       m.get('queue_wait_p50_s') or 0, m.get('queue_wait_p95_s') or 0,
                       m.get('passengers_boarded'))})
    return bool(r.get('passed')), checks


def read_verdict(task, out_dir, returncode, log_tail, timed_out):
    """Return (status, checks, summary) for a finished run.

    status is 'passed', 'failed' or 'error' (no result could be read).
    """
    if task['run_for']:
        ok = timed_out
        return ('passed' if ok else 'failed',
                [{'check': 'ran for the requested time', 'passed': ok,
                  'detail': 'stopped by the timer' if ok else 'ended early (see log)'}],
                'live demo finished' if ok else 'ended before the timer')
    if task['command'] == 'pytest':
        last = next((ln for ln in reversed(log_tail) if 'passed' in ln or 'failed' in ln), '')
        ok = returncode == 0
        return ('passed' if ok else 'failed',
                [{'check': 'pytest', 'passed': ok, 'detail': last.strip('= ')}],
                last.strip('= ') or 'exit code %s' % returncode)
    path = os.path.join(out_dir, task['check'])
    try:
        with open(path) as fh:
            result = json.load(fh)
    except (OSError, ValueError):
        return 'error', [], 'no result file %s (exit code %s, see log)' % (
            task['check'], returncode)
    if task['command'] == 'video_eval':
        ok, checks = _verdict_real_video(result)
    elif task['command'] == 'train':
        ok, checks = _verdict_training(result)
    elif task['id'] == 'S9x':
        ok, checks = _verdict_repeat(result)
    elif task['id'] == 'S8':
        ok, checks = _verdict_episode(result)
    else:
        ok, checks = bool(result.get('passed')), _check_list(result.get('checks'))
    passed = sum(c['passed'] for c in checks)
    return ('passed' if ok else 'failed', checks,
            '%d/%d checks passed' % (passed, len(checks)) if checks else '')


class RunManager:
    """Queue, execute and remember task runs (see module docstring)."""

    def __init__(self, root, out_base='output/ui', popen=subprocess.Popen, clock=time.time):
        self.root = root
        self.out_base = out_base
        self._popen, self._clock = popen, clock
        self._lock = threading.Lock()
        self._wake = threading.Condition(self._lock)
        self._queue = collections.deque()
        self._active = None
        self._proc = None
        self._seq = 0
        self.history = self._load_history()
        self._worker = threading.Thread(target=self._loop, name='cc-runs', daemon=True)
        self._worker.start()

    # ------------------------------------------------------------ public API
    def submit(self, task_id, options=None, source='form'):
        """Validate and queue one run; return its summary. Raises ValueError."""
        task = BY_ID.get(task_id)
        if task is None:
            raise ValueError('unknown task %r' % task_id)
        values = resolve_options(task, options)
        with self._lock:
            self._seq += 1
            run_id = '%s-%s-%02d' % (time.strftime('%m%d-%H%M%S', time.localtime(self._clock())),
                                     task_id, self._seq % 100)
            out_dir = os.path.join(self.out_base, run_id)
            run = {'id': run_id, 'task': task_id, 'title': task['title'],
                   'owner': task['owner'], 'options': values, 'source': source,
                   'status': 'queued', 'queued_at': self._clock(), 'started_at': None,
                   'ended_at': None, 'out_dir': out_dir, 'live': task['live'],
                   'command': build_command(task, values, out_dir), 'checks': [],
                   'summary': '', 'returncode': None,
                   'log': collections.deque(maxlen=LOG_LINES)}
            self._queue.append(run)
            self._wake.notify()
            return self._public(run, log_lines=0)

    def stop(self, run_id=None):
        """Cancel a queued run or stop the active one (None = the active one)."""
        with self._lock:
            for run in list(self._queue):
                if run['id'] == run_id:
                    self._queue.remove(run)
                    run.update(status='cancelled', ended_at=self._clock(),
                               summary='cancelled before it started')
                    self._remember(run)
                    return True
            active = self._active
            if active is None or (run_id not in (None, active['id'])):
                return False
            active['stop_requested'] = True
            proc = self._proc
        if proc is not None:
            threading.Thread(target=self._terminate, args=(proc,), daemon=True).start()
        return True

    def snapshot(self, log_lines=80):
        """Return active run, queue and history for the UI."""
        with self._lock:
            return {'active': self._public(self._active, log_lines) if self._active else None,
                    'queue': [self._public(r, 0) for r in self._queue],
                    'history': list(self.history[-50:])[::-1]}

    def log(self, run_id):
        """Return the full log of a run (from disk once it has ended)."""
        with self._lock:
            if self._active and self._active['id'] == run_id:
                return '\n'.join(self._active['log'])
        path = os.path.join(self.root, self.out_base, run_id, 'log.txt')
        try:
            with open(path, errors='replace') as fh:
                return fh.read()[-200000:]
        except OSError:
            return ''

    # -------------------------------------------------------------- internals
    def _public(self, run, log_lines=80):
        out = {k: v for k, v in run.items() if k not in ('log', 'stop_requested')}
        out['log'] = list(run['log'])[-log_lines:] if log_lines else []
        now = run['ended_at'] or self._clock()
        out['elapsed'] = round(now - run['started_at'], 1) if run['started_at'] else 0.0
        return out

    def _loop(self):
        while True:
            with self._lock:
                while not self._queue:
                    self._wake.wait()
                run = self._queue.popleft()
                run.update(status='running', started_at=self._clock())
                self._active = run
            try:
                self._execute(run)
            except Exception as exc:      # keep the worker alive for the next run
                run.update(status='error', summary='could not run: %s' % exc)
            with self._lock:
                run['ended_at'] = self._clock()
                self._active, self._proc = None, None
                self._remember(run)

    def _execute(self, run):
        task = BY_ID[run['task']]
        out_dir = os.path.join(self.root, run['out_dir'])
        os.makedirs(out_dir, exist_ok=True)
        timer, timed_out = None, threading.Event()
        with open(os.path.join(out_dir, 'log.txt'), 'w') as log_file:
            log_file.write('$ %s\n' % ' '.join(run['command']))
            proc = self._popen(run['command'], cwd=self.root, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, bufsize=1,
                               start_new_session=True)
            with self._lock:
                self._proc = proc
            if task['run_for']:
                def expire():
                    timed_out.set()
                    self._terminate(proc)
                timer = threading.Timer(run['options'][task['run_for']], expire)
                timer.daemon = True
                timer.start()
            for line in proc.stdout:
                line = line.rstrip('\n')
                log_file.write(line + '\n')
                with self._lock:
                    run['log'].append(line)
            proc.wait()
        if timer is not None:
            timer.cancel()
        run['returncode'] = proc.returncode
        if run.get('stop_requested'):
            run.update(status='stopped', summary='stopped by the user')
            return
        status, checks, summary = read_verdict(task, out_dir, proc.returncode,
                                               list(run['log'])[-30:], timed_out.is_set())
        run.update(status=status, checks=checks, summary=summary)

    def _terminate(self, proc):
        for sig, grace in ((signal.SIGINT, STOP_GRACE[0]), (signal.SIGTERM, STOP_GRACE[1]),
                           (signal.SIGKILL, 0.0)):
            if proc.poll() is not None:
                return
            try:
                os.killpg(os.getpgid(proc.pid), sig)
            except (ProcessLookupError, PermissionError, OSError):
                return
            deadline = time.monotonic() + grace
            while grace and time.monotonic() < deadline and proc.poll() is None:
                time.sleep(0.1)

    def _remember(self, run):
        """Append run to the history and save it (caller holds the lock)."""
        entry = {k: v for k, v in self._public(run, 0).items() if k not in ('log', 'command')}
        self.history.append(entry)
        del self.history[:-HISTORY_MAX]
        path = os.path.join(self.root, self.out_base, 'history.json')
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path + '.tmp', 'w') as fh:
                json.dump(self.history, fh, indent=1)
            os.replace(path + '.tmp', path)
        except OSError:
            pass                           # history is a convenience, never fatal

    def _load_history(self):
        try:
            with open(os.path.join(self.root, self.out_base, 'history.json')) as fh:
                data = json.load(fh)
            return data if isinstance(data, list) else []
        except (OSError, ValueError):
            return []
