import pathlib
from ros_introspect.finder import find_package_roots


def test_package_finder():
    folder = pathlib.Path(__file__).parent.parent / 'test_data'
    roots = list(find_package_roots(folder))
    assert len(roots) == 3
