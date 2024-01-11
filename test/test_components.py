
from ros_introspect import find_packages, Package
from ros_introspect.package import PackageFile
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


def test_missing_files():
    pkg = Package(TEST_DATA_FOLDER / 'eleanor' / 'hibachi')
    for subtype in PackageFile.SUBTYPES:
        subtype(pkg.root / 'DOES_NOT_EXIST', pkg)
