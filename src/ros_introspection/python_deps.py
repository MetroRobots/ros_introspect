from .util import get_download_data

PYTHON_DEPS = {}


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
