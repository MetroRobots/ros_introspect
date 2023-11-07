from .package_structure import get_package_structure


class Package:
    def __init__(self, root):
        package_structure = get_package_structure(root)
        self.misc_files = list(package_structure[None].keys()) + list(package_structure['models'].keys())

    def get_test_dependencies(self):
        packages = set()
        packages.update(self.source_code.get_test_dependencies())
        for launch in self.launches:
            if not launch.test:
                continue
            packages.add('rostest')
            packages.update(launch.get_dependencies())
        return packages
