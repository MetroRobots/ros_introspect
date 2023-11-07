from ros_introspect.components.cmake import CommandGroup, Command, ROS_TESTING_FLAGS, is_testing_group

BUILD_TARGET_COMMANDS = ['qt5_wrap_cpp', 'add_library', 'add_executable', 'add_rostest',
                         'target_include_directories', 'add_dependencies', 'target_link_libraries',
                         'set_target_properties', 'ament_target_dependencies']
TEST_COMMANDS = [('group', variable) for variable in ROS_TESTING_FLAGS] + \
                ['catkin_download_test_data',
                 'roslint_cpp', 'roslint_python', 'roslint_add_test',
                 'catkin_add_nosetests', 'catkin_add_gtest', 'ament_add_gtest', 'add_rostest_gtest',
                 'ament_lint_auto_find_test_dependencies']
INSTALL_COMMANDS = ['install', 'catkin_install_python', 'ament_python_install_module', 'ament_python_install_package',
                    'ament_export_include_directories', 'ament_export_libraries',
                    'ament_export_dependencies', 'ament_export_targets', 'pluginlib_export_plugin_description_file']

BASE_ORDERING = ['cmake_minimum_required', 'project',
                 ('group', 'CMAKE_C_STANDARD'), ('group', 'CMAKE_CXX_STANDARD'), ('group', 'CMAKE_COMPILER_IS_GNUCXX'),
                 ('set', 'CMAKE'),
                 'set_directory_properties', 'add_compile_options', 'find_package', 'pkg_check_modules',
                 'moveit_build_options',
                 ('set', 'OTHER'),
                 'catkin_generate_virtualenv', 'catkin_python_setup', 'add_definitions',
                 'add_message_files', 'add_service_files', 'add_action_files', 'rosidl_generate_interfaces',
                 'generate_dynamic_reconfigure_options', 'generate_messages', 'catkin_package', 'catkin_metapackage',
                 BUILD_TARGET_COMMANDS + ['include_directories'],
                 ]


def get_style(cmake):
    """Examine the contents of the cmake parameter and determine the style.

    There are four possible styles:
    1) test_first (where test commands come strictly before install commands)
    2) install_first (where test commands come strictly after install commands)
    3) mixed (where test and install commands are not clearly delineated)
    4) None (where there are only install commands, or only test commands, or neither)
    """
    cats = []
    for content in cmake.contents:
        cat = None
        if isinstance(content, CommandGroup) and is_testing_group(content):
            cat = 'test'
        elif isinstance(content, Command):
            if content.command_name in TEST_COMMANDS:
                cat = 'test'
            elif content.command_name in INSTALL_COMMANDS:
                cat = 'install'
        if cat is None:
            continue

        if len(cats) == 0 or cats[-1] != cat:
            cats.append(cat)

        if len(cats) > 2:
            return 'mixed'
    if len(cats) < 2:
        return None
    first_cat = cats[0]
    return first_cat + '_first'


def get_ordering(style):
    """Given the style, return the correct ordering."""
    if style == 'install_first':
        return BASE_ORDERING + INSTALL_COMMANDS + TEST_COMMANDS + ['ament_package']
    else:
        return BASE_ORDERING + TEST_COMMANDS + INSTALL_COMMANDS + ['ament_package']


def get_ordering_index(command_name, ordering):
    """
    Given a command name, determine the integer index into the ordering.

    The ordering is a list of strings and arrays of strings.

    If the command name matches one of the strings in the inner arrays,
    the index of the inner array is returned.

    If the command name matches one of the other strings, its index is returned.

     Otherwise, the length of the ordering is returned (putting non-matches at the end)
    """
    for i, o in enumerate(ordering):
        if isinstance(o, list):
            if command_name in o:
                return i
        elif command_name == o:
            return i
    if command_name:
        print(f'\tUnsure of ordering for {command_name}')
    return len(ordering)


def get_sort_key(content, anchors, ordering):
    """
    Given a piece of cmake content, return a tuple representing its sort_key.

    The first element of the tuple is the ordering_index of the content.
    The second element is an additional variable used for sorting among elements with the same ordering_index

    Most notably, we want all build commands with a particular library/executable to be grouped together.
    In that case, we use the anchors parameter, which is an ordered list of all the library/executables in the file.
    Then, the second variable is a tuple itself, with the first element being the index of library/executable in the
    anchors list, and the second is an integer representing the canonical order of the build commands.
    """
    if content is None:
        return len(ordering) + 1, None
    index = None
    key = ()
    if content.__class__ == CommandGroup:
        key_token = ()
        for token in content.initial_tag.get_tokens(include_name=True):
            if token == 'NOT':
                continue
            key_token = token
            break
        index = get_ordering_index(('group', key_token), ordering)
    elif content.command_name == 'set':
        token = content.get_tokens(include_name=True)[0]
        if token.startswith('CMAKE_'):
            set_type = 'CMAKE'
        else:
            set_type = 'OTHER'
        index = get_ordering_index(('set', set_type), ordering)
    else:  # Command
        index = get_ordering_index(content.command_name, ordering)
        if content.command_name in BUILD_TARGET_COMMANDS:
            token = content.first_token()
            if token not in anchors:
                anchors.append(token)
            key = anchors.index(token), BUILD_TARGET_COMMANDS.index(content.command_name)
        elif content.command_name == 'include_directories' and 'include_directories' in anchors:
            key = -1, anchors.index('include_directories')
        elif content.command_name == 'find_package':
            token = content.get_tokens()[0]
            if token == 'catkin' or token.startswith('ament_cmake'):
                key = -1, token  # Force ament_cmake/catkin to come first
            else:
                key = 0, token
    return index, key


class CMake:
    def __init__(self):
        self.existing_style = get_style(self)

    def get_insertion_index(self, cmd):
        anchors = self.get_ordered_build_targets()
        ordering = get_ordering(self.get_desired_style())

        new_key = get_sort_key(cmd, anchors, ordering)
        i_index = 0

        for i, content in enumerate(self.contents):
            if isinstance(content, str):
                continue
            key = get_sort_key(content, anchors, ordering)
            if key <= new_key:
                i_index = i + 1
            elif key[0] != len(ordering):
                return i_index
        return len(self.contents)

    def is_metapackage(self):
        return len(self.content_map['catkin_metapackage']) > 0

    def get_libraries(self):
        return list(self.get_source_build_rules('add_library').keys())

    def get_executables(self):
        return list(self.get_source_build_rules('add_executable').keys())

    def get_target_build_rules(self):
        targets = {}
        targets.update(self.get_source_build_rules('add_library'))
        targets.update(self.get_source_build_rules('add_executable'))
        return targets

    def get_ordered_build_targets(self):
        targets = []
        for content in self.contents:
            if content.__class__ != Command:
                continue
            if content.command_name == 'include_directories':
                targets.append('include_directories')
                continue
            elif content.command_name not in BUILD_TARGET_COMMANDS:
                continue
            token = content.first_token()
            if token not in targets:
                targets.append(token)
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

    def get_command_section(self, command_name, section_name):
        """Return the first command that matches the command name and has a matching section name.

        If the section name is not found, return a command with the matching command name
        """
        if len(self.content_map[command_name]) == 0:
            return None, None
        for cmd in self.content_map[command_name]:
            s = cmd.get_section(section_name)
            if s:
                return cmd, s
        return self.content_map[command_name][0], None

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

    def write(self, fn=None):
        if fn is None:
            fn = self.file_path
        with open(fn, 'w') as cmake:
            cmake.write(str(self))
