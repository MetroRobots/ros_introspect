import collections
from enum import IntEnum
import pathlib
import shutil

from betsy_ros import ROSInterface

from .finder import find_package_roots, walk
from .ros_resources import ROSResources
from .util import convert_to_underscore_notation

DependencyType = IntEnum('DependencyType', ['BUILD', 'RUN', 'TEST'])


class PackageFile:
    SUBTYPES = []

    def __init__(self, full_path, package):
        self.full_path = full_path
        self.package = package
        self.rel_fn = full_path.relative_to(package.root)
        self.changed = False

    @classmethod
    def is_singular(cls):
        return False

    @classmethod
    def is_type(cls, path):
        raise NotImplementedError

    @classmethod
    def category_name(cls):
        return cls.__name__

    @classmethod
    def attribute_name(cls):
        return convert_to_underscore_notation(cls.category_name())

    @classmethod
    def needs_share_installation(cls):
        return False

    def get_dependencies(self, dependency_type):
        return set()

    def save(self):
        if not self.changed:
            return
        self.write(self.full_path)
        self.changed = False

    def write(self, output_path):
        # By default, write with no modifications
        if self.full_path != output_path:
            shutil.copy(self.full_path, output_path)

    def __repr__(self):
        return str(self.rel_fn)


class SingularPackageFile(PackageFile):
    @classmethod
    def is_singular(cls):
        return True

    @classmethod
    def is_type(cls, path):
        return cls.category_name() == path.name


class PackageTextFile(PackageFile):
    def __init__(self, full_path, package):
        super().__init__(full_path, package)
        if self.full_path.exists():
            with open(self.full_path, 'r') as f:
                self.contents = f.read()
        else:
            self.contents = None

        self.original_contents = self.contents

    def get_lines(self):
        lines = []
        if self.contents:
            lines = list(self.contents.split('\n'))
        return lines

    def regenerate_contents(self):
        return self.contents

    def force_regeneration(self):
        if self.contents != self.regenerate_contents():
            self.changed = True

    def write(self, output_path):
        with open(output_path, 'w') as f:
            f.write(self.regenerate_contents())


class MiscPackageFile(PackageFile):
    @classmethod
    def is_type(cls, path):
        # As a fallback, everything is a MiscPackageFile
        return True

    @classmethod
    def category_name(cls):
        return 'Other Files'

    @classmethod
    def attribute_name(cls):
        return 'misc_files'

    @classmethod
    def needs_share_installation(cls):
        return True


def package_file(cls):
    """Decorator function to add to static list"""
    PackageFile.SUBTYPES.append(cls)
    return cls


def infer_package_file(path, package):
    for subtype in PackageFile.SUBTYPES:
        if subtype.is_type(path):
            return subtype(path, package)
    return MiscPackageFile(path, package)


class Package:
    def __init__(self, root):
        if isinstance(root, str):
            root = pathlib.Path(root)
        self.root = root
        self.components_by_type = collections.defaultdict(list)
        self.components_by_name = {}

        # Syntactic sugar to allow for direct attribute access
        for subtype in PackageFile.SUBTYPES + [MiscPackageFile]:
            attr_name = subtype.attribute_name()
            if subtype.is_singular():
                setattr(self, attr_name, None)
            else:
                setattr(self, attr_name, self.components_by_type[subtype])

        # Walk all files
        for subpath in walk(root):
            self.add_file(infer_package_file(root / subpath, self))

        # Get Key Properties from Manifest
        assert self.package_xml
        self.name = self.package_xml.name
        self.build_type = self.package_xml.build_type
        self.is_metapackage = self.package_xml.contains_node('metapackage')

        if self.cmakes:
            self.cmake = self.cmakes[0]
            self.is_metapackage = self.is_metapackage or self.cmake.is_metapackage
        else:
            self.cmake = None

        # Update cross-file properties
        if self.cmake and self.source_code:
            self.setup_source_tags()

        # Need Package name to be defined before we update resources
        resources = ROSResources.get()
        resources.packages.add(self.name)
        for interface in self.get_ros_interfaces():
            ros_name = ROSInterface(self.name, interface.type, interface.name)
            resources.add_interface(ros_name)

    @property
    def ros_version(self):
        if self.build_type == 'catkin':
            return 1
        else:
            return 2

    def get_ros_interfaces(self):
        return self.messages + self.services + self.actions

    def add_file(self, package_file):
        subtype = type(package_file)
        self.components_by_type[subtype].append(package_file)
        self.components_by_name[package_file.rel_fn] = package_file

        if subtype.is_singular():
            attr_name = subtype.attribute_name()
            assert getattr(self, attr_name) is None
            setattr(self, attr_name, package_file)

    def remove_file(self, package_file):
        subtype = type(package_file)
        self.components_by_type[subtype].remove(package_file)
        del self.components_by_name[package_file.rel_fn]

        if subtype.is_singular():
            setattr(self, subtype.attribute_name(), None)

        if package_file == self.cmake:
            self.cmake = None

    def __iter__(self):
        for components in self.components_by_type.values():
            yield from components

    def get_dependencies(self, dependency_type):
        deps = set()
        for component in self:
            deps |= component.get_dependencies(dependency_type)
        if self.name in deps:
            deps.remove(self.name)
        return deps

    def setup_source_tags(self):
        for tag, files in self.cmake.get_source_tags().items():
            for rel_fn in files:
                rel_path = pathlib.Path(rel_fn)
                if rel_path in self.components_by_name:
                    self.components_by_name[rel_path].tags.add(tag)
                else:
                    # TODO: Do not raise if rel_fn[0] == '$'
                    # TODO: Do not raise if rel_fn matches ALL_CAPS_PATTERN
                    print(f'Cannot find {rel_fn} in package {self.name}')

    def get_source_by_tags(self, tags, language=None):
        if isinstance(tags, str):
            tags = set([tags])

        tagged = []
        for source_file in self.source_code:
            if language and source_file.language != language:
                continue
            if len(tags.intersection(source_file.tags)) < len(tags):
                continue
            tagged.append(source_file)
        return tagged

    def has_changes(self):
        for component in self:
            if component.changed:
                return True
        return False

    def save(self):
        for component in self:
            component.save()

    def __repr__(self):
        s = '== {} ({})==\n'.format(self.name, self.build_type)
        for subtype in PackageFile.SUBTYPES + [MiscPackageFile]:
            if not self.components_by_type[subtype]:
                continue
            if subtype.is_singular():
                # should only be one
                assert len(self.components_by_type[subtype]) == 1
                single = self.components_by_type[subtype][0]
                s += f'  {single.rel_fn}\n'
            else:
                s += f'  {subtype.category_name()}\n'
                for package_file in self.components_by_type[subtype]:
                    s += f'    {package_file}\n'
        return s


def find_packages(root_folder=pathlib.Path('.')):
    for package_root in find_package_roots(root_folder):
        yield Package(package_root)
