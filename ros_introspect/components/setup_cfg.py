from ..package import SingularPackageFile, package_file
import configparser


@package_file
class SetupCFG(SingularPackageFile):
    def __init__(self, full_path, package):
        super().__init__(full_path, package)

        self.config = configparser.ConfigParser()
        if self.full_path.exists():
            self.config.read(self.full_path)
        else:
            self.changed = True

    def ensure(self, section_name, key, value):
        if not self.config.has_section(section_name):
            self.config.add_section(section_name)
            self.changed = True
        old_val = self.config[section_name].get(key)
        if old_val != value:
            self.config.set(section_name, key, value)
            self.changed = True

    def write(self, output_path):
        with open(output_path, 'w') as f:
            self.config.write(f, space_around_delimiters=False)
