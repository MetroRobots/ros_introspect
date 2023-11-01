from ..package import PackageFile, package_file


@package_file
class DynamicReconfig(PackageFile):
    def __init__(self, full_path, package):
        super().__init__(full_path, package)

    @classmethod
    def is_type(cls, path):
        return path.suffix == '.cfg' and path.parts[-2] == 'cfg'

    def write(self, output_path):
        with open(self.full_path) as rf:
            with open(output_path, 'w') as wf:
                wf.write(rf.read())
