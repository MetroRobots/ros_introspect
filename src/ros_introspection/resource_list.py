import datetime
import os

import requests
from .ros_util import list_packages, list_interfaces

import yaml

DOT_ROS_FOLDER = os.path.expanduser('~/.ros')
PY_DEP_FILENAME = os.path.join(DOT_ROS_FOLDER, 'py_deps.yaml')
LICENSES_FILENAME = os.path.join(DOT_ROS_FOLDER, 'license_info.yaml')

PYTHON_DEPS = {}
LICENSE_INFO = {}


def make_download_attempt(filename, url, json=True, preprocess_fne=None):
    now = datetime.datetime.now()
    if os.path.exists(filename):
        data = yaml.safe_load(open(filename))

        if 'last_download' in data and now - data['last_download'] < datetime.timedelta(days=3):
            # Data is sufficiently new
            return data

    # Attempt a download
    try:
        req = requests.get(url)
        if req.status_code != 200:
            raise RuntimeError('Non-OK Status')
    except requests.exceptions.ConnectionError:
        print(f'Cannot retrieve {filename}')
        return

    if json:
        data = req.json()
    else:
        data = yaml.safe_load(req.text)

    if preprocess_fne:
        data = preprocess_fne(data)

    data['last_download'] = now

    if not os.path.exists(DOT_ROS_FOLDER):
        os.mkdir(DOT_ROS_FOLDER)
    yaml.dump(data, open(filename, 'w'))
    return data


def maybe_download_python_deps():
    global PYTHON_DEPS
    PYTHON_DEPS = make_download_attempt(PY_DEP_FILENAME,
                                        'https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml',
                                        json=False)


def get_python_dependency(key):
    for var in [key, 'python-' + key, 'python3-' + key, key.replace('python-', 'python3-'), key.replace('python-', ''),
                key.replace('python-', '').replace('-', '_')]:
        if var in PYTHON_DEPS:
            return var


maybe_download_python_deps()


def preprocess_licenses(data):
    processed = {}
    for license_dict in data:
        key = license_dict['key']
        del license_dict['key']
        processed[key] = license_dict
    return processed


# Keys available: https://github.com/licensee/licensee/blob/master/spec/fixtures/license-hashes.json
def get_license_info(license_key, get_body=False, allow_uncommon_downloads=False):
    global LICENSE_INFO

    # Download list of common licenses if not present
    if not LICENSE_INFO:
        LICENSE_INFO = make_download_attempt(LICENSES_FILENAME,
                                             'https://api.github.com/licenses',
                                             preprocess_fne=preprocess_licenses)

    if license_key not in LICENSE_INFO:
        if license_key in LICENSE_INFO.get('invalid', []) or not allow_uncommon_downloads:
            return
        else:
            LICENSE_INFO[license_key] = {}

    license_info = LICENSE_INFO[license_key]

    if 'name' not in license_info or (get_body and 'body' not in license_info):
        # Download more information about the specific license
        try:
            print(f'Downloading license: {license_key}')
            req = requests.get(f'https://api.github.com/licenses/{license_key}')
            if req.status_code != 200:
                req = None
        except requests.exceptions.ConnectionError:
            req = None

        if req is None:
            print(f'Cannot retrieve license {license_key}')
            del LICENSE_INFO[license_key]
            if 'invalid' not in LICENSE_INFO:
                LICENSE_INFO['invalid'] = [license_key]
            else:
                LICENSE_INFO['invalid'].append(license_key)
        else:
            license_info.update(req.json())

        yaml.dump(LICENSE_INFO, open(LICENSES_FILENAME, 'w'))

    return license_info


def get_available_licenses():
    global LICENSE_INFO
    if not LICENSE_INFO:
        get_license_info(None)
    return [key for key in LICENSE_INFO.keys() if key not in ['last_download', 'invalid']]


PACKAGES = set(list_packages())
MESSAGES = set()
SERVICES = set()
ACTIONS = set()

for pkg in PACKAGES:
    for mode, ros_set in [('.msg', MESSAGES), ('.srv', SERVICES), ('.action', ACTIONS)]:
        for pkg, gen in list_interfaces(pkg, mode):
            ros_set.add((pkg, gen))


def is_package(pkg):
    return pkg in PACKAGES


def is_message(pkg, msg):
    return (pkg, msg) in MESSAGES


def is_service(pkg, srv):
    return (pkg, srv) in SERVICES
