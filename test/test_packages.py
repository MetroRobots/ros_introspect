from ros_introspect import find_packages, Package
from ros_introspect.package import PackageFile, MiscPackageFile, DependencyType
from test_finder import TEST_DATA_FOLDER, THIS_FILE
import pytest
import pathlib
import tempfile


def test_find_packages():
    pkgs = list(find_packages(TEST_DATA_FOLDER))
    assert len(pkgs) == 4
    for pkg in pkgs:
        print(pkg)


def test_waymond():
    pkg = Package(TEST_DATA_FOLDER / 'waymond')
    assert pkg


def test_kungfu():
    pkg = Package(TEST_DATA_FOLDER / 'eleanor' / 'kungfu')
    assert pkg.name == 'kungfu'
    assert str(pkg) != ''

    assert pathlib.Path('test_file.yaml') in pkg.components_by_name
    assert pathlib.Path('ignore_file.yaml~') not in pkg.components_by_name


def test_meta():
    pkg = Package(TEST_DATA_FOLDER / 'eleanor' / 'eeaao')
    assert pkg


class FakeComponent(PackageFile):
    pass


def test_bad_component():
    with pytest.raises(NotImplementedError):
        FakeComponent.is_type('')
    assert FakeComponent.category_name() == 'FakeComponent'
    assert FakeComponent.attribute_name() == 'fake_component'

    # Keep defaults consistent
    assert FakeComponent.needs_share_installation() is False


def test_all_components():
    folder = TEST_DATA_FOLDER / 'waymond'
    pkg = Package(str(folder))
    for subtype in PackageFile.SUBTYPES + [MiscPackageFile]:
        # Make sure there are no errors raised
        print(subtype.category_name())
        assert subtype.is_type(THIS_FILE) is not None
        assert subtype.needs_share_installation() in [True, False]
        with tempfile.NamedTemporaryFile(mode='w', prefix=str(folder) + '/') as temp:
            p = pathlib.Path(temp.name)
            st = subtype(p, pkg)
            st.changed = True
            st.save()
            assert isinstance(st.get_dependencies(DependencyType.BUILD), set)
