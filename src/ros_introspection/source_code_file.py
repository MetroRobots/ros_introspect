import re

from .resource_list import get_python_dependency, is_package

PKG = r'([^\.;]+)(\.?[^;]*)?'
PYTHON1 = '^import ' + PKG
PYTHON2 = 'from ' + PKG + ' import .*'
CPLUS = re.compile(r'#include\s*[<\\"]([^/]*)/?([^/]*)[>\\"]')          # Zero or one slash
CPLUS2 = re.compile(r'#include\s*[<\\"]([^/]*)/([^/]*)/([^/]*)[>\\"]')  # Two slashes
ROSCPP = re.compile(r'#include\s*<ros/ros.h>')

EXPRESSIONS = [re.compile(PYTHON1), re.compile(PYTHON2), CPLUS, CPLUS2]


class SourceCodeFile:
    def replace_contents(self, contents):
        self.changed_contents = contents
        try:
            self.lines = map(unicode.strip, unicode(contents).split('\n'))
        except NameError:
            # Python3 Case
            self.lines = list(map(str.strip, contents.split('\n')))

    def search_for_patterns(self, patterns):
        matches = []
        contents = self.get_contents()
        for pattern in patterns:
            matches += pattern.findall(contents)
        return matches

    def search_lines_for_patterns(self, patterns):
        matches = []
        for line in self.lines:
            for pattern in patterns:
                m = pattern.search(line)
                if m:
                    matches.append(m.groups())
        return matches

    def search_lines_for_pattern(self, pattern):
        return self.search_lines_for_patterns([pattern])

    def modify_with_patterns(self, patterns, verbose):
        """
        Given a map of patterns, replace all instances in the source code.

        The key in the map (needle) is a regular expression string literal.
        If there are no groups, then the matching string is replaced with the map value.
        If there are groups, then the literals of the form $0, $1, etc in the map value are replaced with the groups
        """
        s = self.get_contents()
        changed = False
        for needle, replacement in patterns.items():
            pattern = re.compile(needle)
            m = pattern.search(s)
            while m:
                this_replacement = replacement
                if len(m.groups()) > 0:
                    for i, chunk in enumerate(m.groups()):
                        key = '$%d' % i
                        this_replacement = this_replacement.replace(key, chunk)
                before, middle, after = s.partition(m.group(0))
                if verbose:
                    print('In %s, replacing %s with %s' % (self.rel_fn, middle, this_replacement))
                s = before + this_replacement + after

                changed = True
                m = pattern.search(s)
            if changed:
                self.replace_contents(s)

    def get_import_packages(self):
        pkgs = set()
        for match in self.search_lines_for_patterns(EXPRESSIONS):
            pkgs.add(match[0])
        if len(self.search_lines_for_pattern(ROSCPP)) > 0:
            pkgs.add('roscpp')
        return sorted(pkgs)

    def get_dependencies(self):
        deps = []
        for pkg in self.get_import_packages():
            if is_package(pkg):
                deps.append(pkg)
        return deps

    def get_external_python_dependencies(self):
        deps = []
        if self.language != 'python':
            return deps

        for pkg in self.get_import_packages():
            p_dep = get_python_dependency(pkg)
            if p_dep:
                deps.append(p_dep)
        return deps

    def __lt__(self, other):
        return self.rel_fn < other.rel_fn
