import collections
from .finder import find_package_roots


class PackageFile:
    SUBTYPES = []

    def __init__(self, full_path, package_root):
        self.full_path = full_path
        self.package_root = package_root
        self.rel_fn = full_path.relative_to(package_root)
        self.changed = False

    @classmethod
    def is_singular(cls):
        return False

    @classmethod
    def is_type(cls, path):
        raise NotImplementedError

    @classmethod
    def category_name(cls):
        raise NotImplementedError

    def save(self):
        if not self.changed:
            return
        self.write(self.full_path)

    def write(self, output_fn):
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


def infer_package_file(path, package_root):
    for subtype in PackageFile.SUBTYPES:
        if subtype.is_type(path):
            return subtype(path, package_root)
    return MiscPackageFile(path, package_root)


class Package:
    def __init__(self, root):
        self.root = root
        self.components_by_type = collections.defaultdict(list)
        self.components_by_name = collections.defaultdict(list)

        # Walk all files
        queue = [root]
        while queue:
            folder = queue.pop(0)
            for subpath in folder.iterdir():
                if subpath.is_dir():
                    queue.append(subpath)
                else:
                    self.add_file(infer_package_file(subpath, self.root))

    def add_file(self, package_file):
        self.components_by_type[type(package_file)].append(package_file)
        self.components_by_name[package_file.rel_fn].append(package_file)

    def save(self):
        for components in self.components_by_type.values():
            for component in components:
                component.save()

    def __repr__(self):
        s = '== {} ==\n'.format(self.root.name)
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
