import os.path
import re

from .source_code_file import SourceCodeFile

ALL_CAPS_PATTERN = re.compile(r'^[A-Z]+$')


class SourceCode:
    def has_header_files(self):
        goal_folder = os.path.join('include', self.pkg_name)
        for source_fn in self.sources:
            if goal_folder in source_fn:
                return True
        return False

    def get_source_by_language(self, language):
        return [source for source in self.sources.values() if source.language == language]

    def get_source_by_tags(self, tags, language=None):
        tagged = []
        for source in self.sources.values():
            if language and source.language != language:
                continue
            if isinstance(tags, str) and tags not in source.tags:
                continue
            if isinstance(tags, set) and len(tags.intersection(source.tags)) < len(tags):
                continue
            tagged.append(source)
        return tagged

    def get_build_dependencies(self):
        packages = set()
        for source in self.sources.values():
            if 'test' in source.tags:
                continue
            packages.update(source.get_dependencies())
        if self.pkg_name in packages:
            packages.remove(self.pkg_name)
        return packages

    def get_external_python_dependencies(self):
        packages = set()
        for source in self.sources.values():
            if 'test' in source.tags:
                continue
            packages.update(source.get_external_python_dependencies())
        return packages

    def get_test_dependencies(self):
        packages = set()
        for source in self.sources.values():
            print(source, source.tags)
            if 'test' not in source.tags:
                continue
            packages.update(source.get_dependencies())

            if source.language == 'python':
                packages.add('ament_cmake_pytest')
        if self.pkg_name in packages:
            packages.remove(self.pkg_name)
        return packages

    def search_for_patterns(self, patterns, per_line=True):
        files = {}
        for source in self.sources.values():
            if per_line:
                matches = source.search_lines_for_patterns(patterns)
            else:
                matches = source.search_for_patterns(patterns)
            if len(matches) != 0:
                files[source.rel_fn] = matches
        return files

    def search_for_pattern(self, pattern, per_line=True):
        return self.search_for_patterns([pattern], per_line)

    def modify_with_patterns(self, patterns, language='c++', verbose=True):
        """
        Given a map of patterns, replace all instances in the package source code with the given language.

        The key in the map is a regular expression string literal.
        If there are no groups, then the matching string is replaced with the map value.
        If there are groups, then the literals of the form $0, $1, etc in the map value are replaced with the groups
        """
        for source in self.get_source_by_language(language):
            source.modify_with_patterns(patterns, verbose)

    def add_source_code(self, root, rel_fn, contents):
        file_path = os.path.join(root, rel_fn)
        with open(file_path, 'w') as f:
            f.write(contents)
        scf = SourceCodeFile(rel_fn, file_path)
        self.sources[rel_fn] = scf
        return scf

    def __repr__(self):
        return '\n'.join(map(str, sorted(self.sources.values())))
