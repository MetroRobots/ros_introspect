from ros_introspect.components.cmake import CommandGroup, Command


class CMake:
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

    def upgrade_minimum_version(self, new_version):
        """Upgrade the CMake version to the new version (specified as a tuple)."""
        for cmd in self.content_map['cmake_minimum_required']:
            section = cmd.get_section('VERSION')
            version = tuple(map(int, section.values[0].split('.')))
            if version < new_version:
                section.values[0] = '.'.join(map(str, new_version))
                cmd.changed = True
