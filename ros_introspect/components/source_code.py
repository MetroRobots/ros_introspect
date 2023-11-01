from ..package import PackageFile, package_file
import os
import re

PYTHON_EXTS = ['.py']
CPP_EXTS = ['.cpp', '.h', '.hpp', '.c', '.cc']
SRC_EXTS = CPP_EXTS + PYTHON_EXTS
INIT_PY = '__init__.py'
PYTHON_TAGS = {
    'std_main': re.compile(r"if\s*__name__\s*==\s*'__main__'"),
    'entry_pt': re.compile(r'(def main\()'),
}


def is_python_hashbang_line(s):
    return s[0:2] == '#!' and 'python' in s


@package_file
class SourceCode(PackageFile):

    def __init__(self, full_path, package):
        super().__init__(full_path, package)
        self.changed_contents = None

        self.lines = list(map(str.strip, self.get_contents().split('\n')))
        if self.full_path.suffix in CPP_EXTS:
            self.language = 'c++'
        elif self.full_path.suffix in PYTHON_EXTS or (len(self.lines) > 0 and is_python_hashbang_line(self.lines[0])):
            self.language = 'python'
        else:
            self.language = 'unknown'

        # Add Tags
        self.tags = set()
        if self.rel_fn.parts and self.rel_fn.parts[0] == 'test':
            self.tags.add('test')

        if self.language == 'python':
            lib_file = self.full_path.parent / INIT_PY
            if lib_file.exists():
                self.tags.add('pylib')
            elif self.is_executable():
                self.tags.add('pyscript')
            for tag, pattern in PYTHON_TAGS.items():
                if self.search_lines_for_pattern(pattern):
                    self.tags.add(tag)

    def get_contents(self):
        if self.changed_contents:
            return self.changed_contents
        return open(self.full_path).read()

    @classmethod
    def is_type(cls, path):
        if path.suffix in SRC_EXTS:
            return True

        with open(path) as f:
            try:
                first_line = f.readline()
                return is_python_hashbang_line(first_line)
            except UnicodeDecodeError:
                pass
        return False

    def is_executable(self):
        return os.access(self.file_path, os.X_OK)

    def write(self, output_path):
        with open(output_path, 'w') as f:
            f.write(self.get_contents())

    def __repr__(self):
        attribs = [self.language] + list(self.tags)
        return '%s (%s)' % (self.rel_fn, ', '.join(attribs))
