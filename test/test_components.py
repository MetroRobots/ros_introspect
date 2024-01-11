
from ros_introspect import find_packages
from test_finder import TEST_DATA_FOLDER
import tempfile


def copy_component_to_temp(component):
    with tempfile.NamedTemporaryFile(mode='w', delete=False) as temp:
        component.write(temp.name)

    original = open(component.full_path, 'r').read()
    s = open(temp.name, 'r').read()

    assert original == s, f'{component.package.name}/{component.rel_fn}'
    temp.close()


def test_component_writes():
    for package in find_packages(TEST_DATA_FOLDER):
        for component in package.components_by_name.values():
            copy_component_to_temp(component)
