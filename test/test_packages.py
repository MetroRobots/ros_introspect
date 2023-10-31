from ros_introspect.package import find_packages
from test_finder import TEST_DATA_FOLDER


def test_find_packages():
    roots = list(find_packages(TEST_DATA_FOLDER))
    assert len(roots) == 3
    print(roots[0])
