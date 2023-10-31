import collections
import os

from .source_code_file import is_python_hashbang_line

KEY = ['package.xml', 'CMakeLists.txt', 'setup.py']
SRC_EXTS = ['.py', '.cpp', '.h', '.hpp', '.c', '.cc']
GENERATORS = ['.msg', '.srv', '.action']
REPO_MARKERS = ['.git', '.svn', 'fake_git_root']


def is_repo_root(folder):
    # Look for repo metadata (or fake_git_root for testing purposes)
    for marker in REPO_MARKERS:
        if os.path.exists(os.path.join(folder, marker)):
            return True
    return False


def is_repo_marker(folder):
    for marker in REPO_MARKERS:
        if marker in folder:
            return True
    return False


def get_repo_root(package):
    repo_root = os.path.abspath(package.root)

    while not is_repo_root(repo_root):
        parent_dir = os.path.abspath(os.path.join(repo_root, os.pardir))
        if repo_root == parent_dir:
            raise RuntimeError('Cannot find repo root: ' + str(package.root))

        repo_root = parent_dir
    return repo_root


def get_filetype_by_contents(filename, ext):
    with open(filename) as f:
        try:
            first_line = f.readline()
        except UnicodeDecodeError:
            return
        if is_python_hashbang_line(first_line):
            return 'source'
        elif '<launch' in first_line:
            return 'launch'
        elif ext == '.xml' and ('<library' in first_line or '<class_libraries' in first_line):
            return 'plugin_config'


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
            elif rel_fn.endswith('.launch.py'):
                structure['launchpy'][rel_fn] = full
            elif ext == '.launch':
                structure['launch'][rel_fn] = full
            elif ext in SRC_EXTS:
                structure['source'][rel_fn] = full
            elif ext in GENERATORS:
                structure['generators'][rel_fn] = full
            elif ext == '.cfg' and 'cfg/' in full:
                structure['cfg'][rel_fn] = full
            elif ext in ['.urdf', '.xacro']:
                structure['urdf'][rel_fn] = full
            else:
                structure[get_filetype_by_contents(full, ext)][rel_fn] = full
    return structure
