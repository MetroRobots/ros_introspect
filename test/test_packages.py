from ros_introspect import find_packages, Package, ROSResources
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
    manifest = pkg.package_xml
    assert pkg.ros_version == 1
    assert pkg.name == 'waymond'
    assert not pkg.has_changes()
    assert len(pkg.get_dependencies(DependencyType.BUILD)) == 2
    assert manifest.name == 'waymond'
    assert manifest.xml_format == 1
    assert manifest.std_tab == 2
    assert str(manifest) == 'package.xml'

    pkg.save()

    people = manifest.get_people()
    assert len(people) == 1
    assert manifest.get_license() == 'BSD 3-clause'

    assert len(pkg.get_ros_interfaces()) == 4

    resources = ROSResources.get()
    assert resources.is_package('waymond')
    assert resources.is_message('waymond', 'FannyPack')
    assert not resources.is_service('waymond', 'FannyPack')
    assert resources.is_service('waymond', 'BeKind')
    assert not resources.is_service('waymond', 'Laundry')
    assert resources.is_action('waymond', 'Laundry')
    assert resources.is_action('waymond', 'Taxes')


def test_kungfu():
    pkg = Package(TEST_DATA_FOLDER / 'eleanor' / 'kungfu')
    manifest = pkg.package_xml
    assert pkg.ros_version == 2
    assert pkg.name == 'kungfu'
    assert manifest.name == 'kungfu'
    assert manifest.xml_format == 2
    assert manifest.std_tab == 4
    assert str(pkg) != ''

    assert pathlib.Path('test_file.yaml') in pkg.components_by_name
    assert pathlib.Path('ignore_file.yaml~') not in pkg.components_by_name


def test_meta():
    pkg = Package(TEST_DATA_FOLDER / 'eleanor' / 'eeaao')
    assert pkg.ros_version == 1
    assert pkg.is_metapackage


def test_source_code():
    resources = ROSResources.get()
    resources.packages.add('rclcpp')

    pkg = Package(TEST_DATA_FOLDER / 'eleanor' / 'hibachi')
    assert len(pkg.get_source_by_tags('library')) == 1
    assert len(pkg.get_source_by_tags(set(), language='python')) == 0

    assert pkg.get_dependencies(DependencyType.BUILD)


def test_bad_case():
    pkg = Package(TEST_DATA_FOLDER / 'jobu' / 'kpop')
    manifest = pkg.package_xml
    assert pkg.ros_version == 1
    assert pkg.name is None
    assert manifest.name is None
    assert manifest.xml_format == 1
    assert manifest.std_tab == 0
    assert manifest.get_license() is None

    manifest.force_regeneration()
    with tempfile.NamedTemporaryFile(mode='w', delete=False) as temp:
        manifest.write(temp.name)
    s = open(temp.name, 'r').read()
    assert '<description/>' in s
    temp.close()


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


def test_editing():
    pkg = Package(TEST_DATA_FOLDER / 'eleanor' / 'hibachi')
    assert len(pkg.components_by_name) == 4
    assert not pkg.has_changes()

    pkg.remove_file(pkg.cmake)
    assert len(pkg.components_by_name) == 3
    pkg.package_xml.set_license('Proprietary')
    assert pkg.has_changes()

    pkg.remove_file(pkg.package_xml)
