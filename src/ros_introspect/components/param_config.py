from ..package import PackageFile, package_file
from .misc_config import MiscConfig
import ruamel.yaml

ruamel_yaml = ruamel.yaml.YAML()


@package_file
class ParamConfig(PackageFile):
    def __init__(self, full_path, package):
        super().__init__(full_path, package)
        self.contents = ruamel_yaml.load(open(full_path))

    @classmethod
    def is_type(cls, path):
        return path.suffix == '.yaml' and not MiscConfig.is_type(path)

    @classmethod
    def needs_share_installation(cls):
        return True

    def write(self, output_path):
        with open(output_path, 'w') as f:
            ruamel_yaml.dump(self.contents, f)
