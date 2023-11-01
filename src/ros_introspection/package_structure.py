import collections
import os

from ros_introspect.finder import is_repo_marker

KEY = ['CMakeLists.txt']


def get_package_structure(pkg_root):
    structure = collections.defaultdict(dict)

    for root, dirs, files in os.walk(pkg_root):
        if is_repo_marker(root):
            continue
        for fn in files:
            ext = os.path.splitext(fn)[-1]
            full = '%s/%s' % (root, fn)
            rel_fn = full.replace(pkg_root + '/', '')

            if fn[-1] == '~' or fn[-4:] == '.pyc':
                continue
            if fn in KEY:
                structure['key'][rel_fn] = full
            elif ext == '.cfg' and 'cfg/' in full:
                structure['cfg'][rel_fn] = full

    return structure
