REPO_MARKERS = ['.git', '.svn', 'fake_git_root']


def is_repo_root(folder):
    # Look for repo metadata (or fake_git_root for testing purposes)
    for marker in REPO_MARKERS:
        if (folder / marker).exists():
            return True
    return False


def is_repo_marker(folder):
    return folder.name in REPO_MARKERS


def find_repo_root(path):
    if path.is_file():
        path = path.parent

    while not is_repo_root(path):
        parent_dir = path.parent
        if path == parent_dir:
            raise RuntimeError(f'Cannot find repo root: {path}')

        path = parent_dir
    return path


def is_package_root(folder):
    if not (folder / 'package.xml').exists():
        return False

    if (folder / 'COLCON_IGNORE').exists():
        return False

    return True


def find_package_roots(root_folder):
    if is_package_root(root_folder):
        yield root_folder
    else:
        for subfolder in root_folder.iterdir():
            if subfolder.is_dir() and not is_repo_marker(subfolder):
                yield from find_package_roots(subfolder)


def find_sibling_package_roots(package_folder):
    parent_path = package_folder.parent
    for subfolder in parent_path.iterdir():
        if not subfolder.is_dir():
            continue
        if is_package_root(subfolder) and subfolder != package_folder:
            yield subfolder
