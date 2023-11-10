from ..package import PackageFile, package_file


@package_file
class Documentation(PackageFile):
    @classmethod
    def is_type(cls, path):
        if path.suffix in ['.rst', '.md']:
            return True
        if 'doc' in path.parts:
            return True
        if path.stem.lower() == 'license':
            return True
        return False
