from ros_introspect import find_packages, Package
from ros_introspect.package import PackageFile
from test_finder import TEST_DATA_FOLDER
import pytest
import tempfile


def test_find_packages():
    pkgs = list(find_packages(TEST_DATA_FOLDER))
    assert len(pkgs) == 4


def test_waymond():
    pkg = Package(TEST_DATA_FOLDER / 'waymond')
    manifest = pkg.package_xml
    assert pkg.name == 'waymond'
    assert manifest.name == 'waymond'
    assert manifest.xml_format == 1
    assert manifest.std_tab == 2
    assert str(manifest) == 'package.xml'

    pkg.save()

    people = manifest.get_people()
    assert len(people) == 1
    assert manifest.get_license() == 'BSD 3-clause'


def test_kungfu():
    pkg = Package(TEST_DATA_FOLDER / 'eleanor' / 'kungfu')
    manifest = pkg.package_xml
    assert pkg.name == 'kungfu'
    assert manifest.name == 'kungfu'
    assert manifest.xml_format == 2
    assert manifest.std_tab == 4
    assert str(pkg) != ''

    # Test Write
    with tempfile.NamedTemporaryFile(mode='w', delete=False) as temp:
        manifest.write(temp.name)

    original = open(manifest.full_path).read()
    s = open(temp.name).read()
    assert original == s
    temp.close()

    assert not manifest.is_metapackage()


def test_meta():
    pkg = Package(TEST_DATA_FOLDER / 'eleanor' / 'eeaao')
    # assert pkg.is_metapackage() TODO: Implement when we have CMake
    assert pkg.package_xml.is_metapackage()


def test_bad_case():
    pkg = Package(TEST_DATA_FOLDER / 'jobu' / 'kpop')
    manifest = pkg.package_xml
    assert pkg.name is None
    assert manifest.name is None
    assert manifest.xml_format == 1
    assert manifest.std_tab == 0
    assert manifest.get_license() is None


def test_bad_case2():
    with pytest.raises(RuntimeError) as e_info:
        Package(TEST_DATA_FOLDER / 'fake_git_root' / 'racacoonie')

    assert 'Unable to determine' in str(e_info)


def test_bad_case3():
    with pytest.raises(RuntimeError) as e_info:
        Package(TEST_DATA_FOLDER / 'fake_git_root' / 'chad')

    assert 'Too many valid buildtool' in str(e_info)


class FakeComponent(PackageFile):
    pass


def test_bad_component():
    with pytest.raises(NotImplementedError):
        FakeComponent.is_type('')
    with pytest.raises(NotImplementedError):
        FakeComponent.category_name()

    fc = FakeComponent(TEST_DATA_FOLDER / 'jobu' / 'kpop', TEST_DATA_FOLDER)
    with pytest.raises(NotImplementedError):
        fc.write('')
