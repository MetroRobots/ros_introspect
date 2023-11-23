from ros_introspect.components.package_xml import FORMAT_3_HEADER
from clean_ros.cleaners.package_xml import replace_package_set


class PackageXML:
    def upgrade(self, new_format=2, quiet=True):
        if self.xml_format == new_format:
            if not quiet:
                print('%s already in format %d!' % (self.name, self.xml_format))
            return

        if new_format not in [2, 3]:
            raise RuntimeError('Unknown PackageXML version: ' + repr(new_format))

        if self.xml_format == 1:
            if not quiet:
                print('Converting {} from version {} to 2'.format(self.name, self.xml_format))
            self._format = 2
            self.root.setAttribute('format', '2')
            replace_package_set(self, ['build_depend', 'run_depend'], 'depend')
            replace_package_set(self, ['run_depend'], 'exec_depend')

        if new_format == 3:
            if not quiet:
                print('Converting {} from version {} to 3'.format(self.name, self.xml_format))
            self._format = 3
            self.root.setAttribute('format', '3')
            self.header = FORMAT_3_HEADER

        self.changed = True
