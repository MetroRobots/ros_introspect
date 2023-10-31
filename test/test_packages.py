from ros_introspect import find_packages
from test_finder import TEST_DATA_FOLDER


def test_find_packages():
    pkgs = list(find_packages(TEST_DATA_FOLDER))
    assert len(pkgs) == 3
