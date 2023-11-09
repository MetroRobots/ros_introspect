from ..package import PackageFile, package_file


@package_file
class Documentation(PackageFile):
    @classmethod
    def is_type(cls, path):
        if path.suffix in ['.rst', '.md']:
            return True
        if 'doc' in path.parts:
            return True
        return False

    def write(self, output_path):
        with open(self.full_path) as rf:
            with open(output_path, 'w') as wf:
                wf.write(rf.read())
