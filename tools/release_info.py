"""Record what a release was built and measured with (run inside Docker).

    docker/hub.sh run 'python3 tools/release_info.py'

Writes two files under evidence/release/:

  versions.txt      commit, base image digest, OS, ROS distribution and RMW,
                    Python, and the exact version of every package the
                    project uses (assignment 24.1: "dependency versions")
  MANIFEST.sha256   SHA-256, size and path of every evidence, model, result
                    and dataset file, so a reader can tell that the files
                    they hold are the ones the report was written from

Both are plain text. A missing optional file (the raw lab video and the
detector weights are not in git) is listed as MISSING rather than skipped,
so an incomplete package is visible.
"""
import hashlib
import os
import platform
import re
import subprocess
import sys

OUT = 'evidence/release'
DEB_PACKAGES = [
    'ros-jazzy-ros-base', 'ros-jazzy-rclpy', 'ros-jazzy-launch', 'ros-jazzy-launch-ros',
    'ros-jazzy-launch-testing', 'ros-jazzy-rosbag2', 'ros-jazzy-rmw-fastrtps-cpp',
    'ros-jazzy-ament-flake8', 'ros-jazzy-ament-pep257', 'ros-jazzy-foxglove-bridge',
    'python3-colcon-core', 'python3-numpy', 'python3-sklearn', 'python3-matplotlib',
    'python3-pytest', 'python3-yaml', 'python3-opencv', 'python3-reportlab', 'ffmpeg',
]
PY_MODULES = ['numpy', 'sklearn', 'cv2', 'matplotlib', 'yaml', 'reportlab', 'pytest', 'rclpy']
HASH_ROOTS = ['evidence', 'src/smart_hub/results', 'src/smart_hub/models', 'models',
              'src/smart_hub/scenarios', 'src/smart_hub/config']
REQUIRED_FILES = ['videos/lab/IMG_0572.MOV', 'models/yolov4-tiny/yolov4-tiny.cfg',
                  'models/yolov4-tiny/yolov4-tiny.weights']


def sh(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=30).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ''


def module_version(name):
    try:
        mod = __import__(name)
    except Exception as exc:            # recorded, never fatal
        return 'NOT IMPORTABLE (%s)' % type(exc).__name__
    return str(getattr(mod, '__version__', 'unknown'))


def versions():
    with open('docker/Dockerfile') as fh:
        digest = re.search(r'^FROM (\S+)', fh.read(), re.M).group(1)
    with open('/etc/os-release') as fh:
        os_name = dict(ln.strip().split('=', 1) for ln in fh if '=' in ln).get(
            'PRETTY_NAME', '').strip('"')
    lines = ['# written by tools/release_info.py; do not edit by hand',
             'commit %s' % os.environ.get('GIT_COMMIT', 'unknown'),
             'base_image %s' % digest,
             'os %s' % os_name,
             'machine %s' % platform.machine(),
             'kernel %s' % platform.release(),
             'ros_distro %s' % os.environ.get('ROS_DISTRO', 'unknown'),
             'rmw %s' % (os.environ.get('RMW_IMPLEMENTATION') or 'rmw_fastrtps_cpp (default)'),
             'ros_domain_id %s' % os.environ.get('ROS_DOMAIN_ID', 'unset'),
             'python %s' % sys.version.split()[0], '', '# Debian packages']
    for pkg in DEB_PACKAGES:
        ver = sh(['dpkg-query', '-W', '-f=${Version}', pkg]) or 'NOT INSTALLED'
        lines.append('%s %s' % (pkg, ver))
    lines += ['', '# Python modules as imported']
    lines += ['%s %s' % (m, module_version(m)) for m in PY_MODULES]
    return lines


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for block in iter(lambda: fh.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def manifest():
    paths = {p for p in REQUIRED_FILES if os.path.isfile(p)}
    for root in HASH_ROOTS:
        for folder, _, files in os.walk(root):
            if folder.startswith(OUT) or '__pycache__' in folder:
                continue
            paths.update(os.path.join(folder, f) for f in files if f != '.DS_Store')
    lines = ['%s  %12d  %s' % (sha256(p), os.path.getsize(p), p) for p in sorted(paths)]
    lines += ['%s  %12s  %s' % ('MISSING'.ljust(64), '-', p)
              for p in REQUIRED_FILES if not os.path.isfile(p)]
    return lines


def main():
    os.makedirs(OUT, exist_ok=True)
    for name, lines in (('versions.txt', versions()), ('MANIFEST.sha256', manifest())):
        with open(os.path.join(OUT, name), 'w') as fh:
            fh.write('\n'.join(lines) + '\n')
        sys.stdout.write('wrote %s/%s (%d lines)\n' % (OUT, name, len(lines)))


if __name__ == '__main__':
    main()
