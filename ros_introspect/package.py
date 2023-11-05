import collections
import pathlib
from .finder import find_package_roots, is_repo_marker
from enum import IntEnum

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
        return cls.category_name().replace('.', '_').lower()

    def get_dependencies(self, dependency_type):
        return set()

    def save(self):
        if not self.changed:
            return
        self.write(self.full_path)

    def write(self, output_path):
        raise NotImplementedError

    def __repr__(self):
        return str(self.rel_fn)


class SingularPackageFile(PackageFile):
    @classmethod
    def is_singular(cls):
        return True

    @classmethod
    def is_type(cls, path):
        return cls.category_name() == path.name


class MiscPackageFile(PackageFile):
    @classmethod
    def category_name(cls):
        return 'Other Files'


def package_file(cls):
    """Decorator function to add to static list"""
    PackageFile.SUBTYPES.append(cls)


def infer_package_file(path, package):
    for subtype in PackageFile.SUBTYPES:
        if subtype.is_type(path):
            return subtype(path, package)
    return MiscPackageFile(path, package)


class Package:
    def __init__(self, root):
        self.root = root
        self.components_by_type = collections.defaultdict(list)
        self.components_by_name = collections.defaultdict(list)

        # Syntactic sugar to allow for direct attribute access
        for subtype in PackageFile.SUBTYPES:
            attr_name = subtype.attribute_name()
            print(attr_name)
            if subtype.is_singular():
                setattr(self, attr_name, None)
            else:
                setattr(self, attr_name, self.components_by_type[subtype])

        # Walk all files
        queue = [root]
        while queue:
            folder = queue.pop(0)
            for subpath in folder.iterdir():
                if subpath.is_dir():
                    # Folder
                    if is_repo_marker(subpath):
                        continue
                    queue.append(subpath)
                else:
                    # File
                    if subpath.suffix == '.pyc' or subpath.suffix.endswith('~'):
                        continue
                    self.add_file(infer_package_file(subpath, self))

        # Get Key Properties from Manifest
        assert self.package_xml
        self.name = self.package_xml.name
        self.build_type = self.package_xml.build_type

    @property
    def ros_version(self):
        if self.build_type == 'catkin':
            return 1
        else:
            return 2

    def add_file(self, package_file):
        subtype = type(package_file)
        self.components_by_type[subtype].append(package_file)
        self.components_by_name[package_file.rel_fn].append(package_file)

        if subtype.is_singular():
            attr_name = subtype.attribute_name()
            assert getattr(self, attr_name) is None
            setattr(self, attr_name, package_file)

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


def find_packages(root_folder):
    for package_root in find_package_roots(root_folder):
        yield Package(package_root)


def print_packages():
    current_folder = pathlib.Path('.')
    for package in find_packages(current_folder):
        print(package)
