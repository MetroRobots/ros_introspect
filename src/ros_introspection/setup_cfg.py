import configparser
import os


class SetupCFG:
    def __init__(self, file_path):
        self.file_path = file_path
        self.config = configparser.ConfigParser()
        if os.path.exists(self.file_path):
            self.changed = False
            self.config.read(self.file_path)
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

    def write(self):
        if not self.changed:
            return
        with open(self.file_path, 'w') as f:
            self.config.write(f, space_around_delimiters=False)
