from ros_introspect.components.cmake import CommandGroup, Command
from clean_ros.cmake_ordering import get_style, get_ordering, get_sort_key


class CMake:
    def __init__(self):
        self.existing_style = get_style(self)

    def get_libraries(self):
        return list(self.get_source_build_rules('add_library').keys())

    def get_executables(self):
        return list(self.get_source_build_rules('add_executable').keys())

    def get_target_build_rules(self):
        targets = {}
        targets.update(self.get_source_build_rules('add_library'))
        targets.update(self.get_source_build_rules('add_executable'))
        return targets

    def get_test_section(self, create_if_needed=False):
        sections = self.get_test_sections()
        if len(sections) > 0:
            return sections[0]
        if not create_if_needed:
            return None

        # Create Test Section
        initial_cmd = Command('if')
        initial_cmd.add_section('CATKIN_ENABLE_TESTING')

        test_contents = CMake(initial_contents=['\n'], depth=self.depth + 1)

        final_cmd = Command('endif')

        cg = CommandGroup(initial_cmd, test_contents, final_cmd)
        self.add_command(cg)
        return cg.sub

    def get_clusters(self, desired_style):
        """Return a list of clusters where each cluster is an array of strings with a Command/CommandGroup at the end.

        The clusters are sorted according to the desired style.
        The strings are grouped at the beginning to maintain the newlines and indenting before each Command.
        """
        anchors = self.get_ordered_build_targets()
        ordering = get_ordering(desired_style)
        clusters = []
        current = []
        for content in self.contents:
            current.append(content)
            if isinstance(content, str):
                continue
            key = get_sort_key(content, anchors, ordering)
            clusters.append((key, current))
            current = []
        if len(current) > 0:
            clusters.append((get_sort_key(None, anchors, ordering), current))

        return [kv[1] for kv in sorted(clusters, key=lambda kv: kv[0])]

    def get_desired_style(self, default_style=None):
        """Determine which style to use, install_first or test_first.

        If the default style is one of those two, use it
        """
        if default_style in ['install_first', 'test_first']:
            desired_style = default_style
        elif default_style is not None:
            raise RuntimeError('Configured default cmake style "{}"'
                               ' is not install_first or test_first'.format(default_style))
        elif self.existing_style in ['install_first', 'test_first']:
            desired_style = self.existing_style
        else:
            # Otherwise, do test first
            desired_style = 'test_first'

        return desired_style

    def enforce_ordering(self, default_style=None):
        desired_style = self.get_desired_style(default_style)
        clusters = self.get_clusters(desired_style)
        self.contents = []
        for contents in clusters:
            self.contents += contents

        for group in self.content_map['group']:
            group.sub.enforce_ordering(default_style)

    def upgrade_minimum_version(self, new_version):
        """Upgrade the CMake version to the new version (specified as a tuple)."""
        for cmd in self.content_map['cmake_minimum_required']:
            section = cmd.get_section('VERSION')
            version = tuple(map(int, section.values[0].split('.')))
            if version < new_version:
                section.values[0] = '.'.join(map(str, new_version))
                cmd.changed = True
