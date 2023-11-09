from ros_introspect.util import get_download_data, CACHE_FOLDER

import requests
import yaml

PYTHON_DEPS = {}
LICENSE_INFO = {}
LICENSES_FILENAME = 'license_info.yaml'


def maybe_download_python_deps():
    global PYTHON_DEPS
    PYTHON_DEPS = get_download_data(
        'https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml', 'py_deps.yaml')


def get_python_dependency(key):
    for var in [key, 'python-' + key, 'python3-' + key, key.replace('python-', 'python3-'), key.replace('python-', ''),
                key.replace('python-', '').replace('-', '_')]:
        if var in PYTHON_DEPS:
            return var


maybe_download_python_deps()


def format_license_info(req):
    processed = {}
    for license_dict in req.json():
        key = license_dict['key']
        del license_dict['key']
        processed[key] = license_dict
    return processed


# Keys available: https://github.com/licensee/licensee/blob/master/spec/fixtures/license-hashes.json
def get_license_info(license_key, get_body=False, allow_uncommon_downloads=False):
    global LICENSE_INFO

    # Download list of common licenses if not present
    if not LICENSE_INFO:
        LICENSE_INFO = get_download_data('https://api.github.com/licenses', LICENSES_FILENAME, format_license_info)

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

        yaml.dump(LICENSE_INFO, open(CACHE_FOLDER / LICENSES_FILENAME, 'w'))

    return license_info


def get_available_licenses():
    global LICENSE_INFO
    if not LICENSE_INFO:
        get_license_info(None)
    return [key for key in LICENSE_INFO.keys() if key not in ['last_download', 'invalid']]
