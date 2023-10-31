import pathlib
import pytest
from ros_introspect.finder import find_package_roots, is_repo_root, find_repo_root, find_sibling_package_roots

THIS_FILE = pathlib.Path(__file__)
ROOT_FOLDER = THIS_FILE.parent.parent
TEST_DATA_FOLDER = ROOT_FOLDER / 'test_data'


def test_package_finder():
    roots = list(find_package_roots(TEST_DATA_FOLDER))
    assert len(roots) == 3


def test_repo_root():
    assert not is_repo_root(THIS_FILE)
    assert not is_repo_root(THIS_FILE.parent)
    assert is_repo_root(ROOT_FOLDER)

    assert find_repo_root(THIS_FILE) == ROOT_FOLDER
    with pytest.raises(RuntimeError):
        find_repo_root(pathlib.Path('/tmp'))


def test_siblings():
    waymond = TEST_DATA_FOLDER / 'waymond'
    w_sibs = list(find_sibling_package_roots(waymond))
    assert len(w_sibs) == 0

    hibachi = TEST_DATA_FOLDER / 'eleanor' / 'hibachi'
    e_sibs = list(find_sibling_package_roots(hibachi))
    assert len(e_sibs) == 1
    assert e_sibs[0].name == 'kungfu'
