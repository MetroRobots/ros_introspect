import collections
import os

from ros_introspect.finder import is_repo_marker


def get_package_structure(pkg_root):
    structure = collections.defaultdict(dict)

    for root, dirs, files in os.walk(pkg_root):
        if is_repo_marker(root):
            continue
        for fn in files:
            if fn[-1] == '~' or fn[-4:] == '.pyc':
                continue

    return structure
