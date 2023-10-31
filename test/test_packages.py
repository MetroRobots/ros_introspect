from ros_introspect import find_packages, Package
from test_finder import TEST_DATA_FOLDER
import pytest
import tempfile


def test_find_packages():
    pkgs = list(find_packages(TEST_DATA_FOLDER))
    assert len(pkgs) == 3


def test_waymond():
    pkg = Package(TEST_DATA_FOLDER / 'waymond')
    manifest = pkg.package_xml
    assert pkg.name == 'waymond'
    assert manifest.name == 'waymond'
    assert manifest.xml_format == 1
    assert manifest.std_tab == 2


def test_kungfu():
    pkg = Package(TEST_DATA_FOLDER / 'eleanor' / 'kungfu')
    manifest = pkg.package_xml
    assert pkg.name == 'kungfu'
    assert manifest.name == 'kungfu'
    assert manifest.xml_format == 2
    assert manifest.std_tab == 4

    # Test Write
    with tempfile.NamedTemporaryFile(mode='w', delete=False) as temp:
        manifest.write(temp.name)

    original = open(manifest.full_path).read()
    s = open(temp.name).read()
    assert original == s
    temp.close()


def test_bad_case():
    pkg = Package(TEST_DATA_FOLDER / 'jobu' / 'kpop')
    manifest = pkg.package_xml
    assert pkg.name is None
    assert manifest.name is None
    assert manifest.xml_format == 1
    assert manifest.std_tab == 0


def test_bad_case2():
    with pytest.raises(RuntimeError) as e_info:
        Package(TEST_DATA_FOLDER / 'fake_git_root' / 'racacoonie')

    assert 'Unable to determine' in str(e_info)


def test_bad_case3():
    with pytest.raises(RuntimeError) as e_info:
        Package(TEST_DATA_FOLDER / 'fake_git_root' / 'chad')

    assert 'Too many valid buildtool' in str(e_info)
