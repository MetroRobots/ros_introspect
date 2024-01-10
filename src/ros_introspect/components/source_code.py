from ..package import PackageTextFile, DependencyType, package_file
from ..ros_resources import ROSResources
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

# Import/Include patterns
_PYTHON_PKG_PATTERN = r'([^\.;]+)(\.?[^;]*)?'
PYTHON_IMPORT0 = re.compile(r'^import ' + _PYTHON_PKG_PATTERN)
PYTHON_IMPORT1 = re.compile('from ' + _PYTHON_PKG_PATTERN + ' import .*')

_NOT_A_SLASH = r'([^/]*)'
_CPP_INITIAL_INCLUDE = r'#include\s*[<\\"]'
_CPP_END_INCLUDE = r'[>\\"]'
# Zero slashes
CPP_INCLUDE0 = re.compile(_CPP_INITIAL_INCLUDE + _NOT_A_SLASH + _CPP_END_INCLUDE)  # Zero slashes
# One slash
CPP_INCLUDE1 = re.compile(_CPP_INITIAL_INCLUDE + _NOT_A_SLASH + '/' + _NOT_A_SLASH + _CPP_END_INCLUDE)
# Two slashes
CPP_INCLUDE2 = re.compile(_CPP_INITIAL_INCLUDE + _NOT_A_SLASH + '/' +
                          _NOT_A_SLASH + '/' + _NOT_A_SLASH + _CPP_END_INCLUDE)

IMPORT_PATTERNS = {
    'python': [PYTHON_IMPORT0, PYTHON_IMPORT1],
    'c++': [CPP_INCLUDE0, CPP_INCLUDE1, CPP_INCLUDE2]
}

SPECIAL_CASE_DEPENDENCIES = {
    'roscpp': re.compile(r'#include\s*<ros/ros.h>')
}


def is_python_hashbang_line(s):
    return s[0:2] == '#!' and 'python' in s


@package_file
class SourceCode(PackageTextFile):

    def __init__(self, full_path, package):
        super().__init__(full_path, package)
        self.resources = ROSResources.get()

        self.lines = self.get_lines()
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
            if self.full_path.name == INIT_PY:
                self.tags.add('init.py')
            lib_file = self.full_path.parent / INIT_PY
            if lib_file.exists():
                self.tags.add('pylib')
            elif self.is_executable():
                self.tags.add('pyscript')
            for tag, pattern in PYTHON_TAGS.items():
                if self.search_lines_for_pattern(pattern):
                    self.tags.add(tag)
        elif self.language == 'c++':
            parts = self.rel_fn.parts
            if len(parts) > 1 and parts[0] == 'include':
                self.tags.add('header')

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

    def search_lines_for_patterns(self, patterns):
        # TODO: Handle multi-line patterns
        matches = []
        for line in self.lines:
            for pattern in patterns:
                m = pattern.search(line)
                if m:
                    matches.append(m.groups())
        return matches

    def search_lines_for_pattern(self, pattern):
        return self.search_lines_for_patterns([pattern])

    def get_import_packages(self):
        pkgs = set()
        patterns = IMPORT_PATTERNS.get(self.language)
        if not patterns:
            return pkgs

        for match in self.search_lines_for_patterns(patterns):
            pkgs.add(match[0])

        return pkgs

    def get_dependencies(self, dependency_type):
        deps = set()
        if dependency_type == DependencyType.TEST and 'test' not in self.tags:
            return deps

        if self.language == 'python' and 'test' in self.tags and self.package.ros_version == 2:
            deps.add('ament_cmake_pytest')

        for pkg in self.get_import_packages():
            if self.resources.is_package(pkg):
                deps.add(pkg)

        for dep, pattern in SPECIAL_CASE_DEPENDENCIES.items():
            if self.search_lines_for_pattern(pattern):
                deps.add(dep)
        return deps

    def is_executable(self):
        return os.access(self.full_path, os.X_OK)

    def __repr__(self):
        attribs = [self.language] + list(self.tags)
        return '%s (%s)' % (self.rel_fn, ', '.join(attribs))
