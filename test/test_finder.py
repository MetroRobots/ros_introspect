import pathlib
import ros_introspect

THIS_FILE = pathlib.Path(__file__)
ROOT_FOLDER = THIS_FILE.parent.parent


def test_package_finder():
    print(ROOT_FOLDER)
    print(ros_introspect)
