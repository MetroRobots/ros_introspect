from ..package import PackageFile, package_file, DependencyType
import re

AT_LEAST_THREE_DASHES = re.compile(r'^\-{3,}\r?$')
FIELD_LINE = re.compile(r'\s*([\w_/]+)(\[\d*\])?\s+([\w_]+)\s*?(=.*)?(\s*\#.*)?$', re.DOTALL)
PRIMITIVES = ['bool', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64',
              'float32', 'float64', 'string', 'time', 'duration']


class InterfaceField:
    def __init__(self, field_type, is_array, name, value):
        self.type = field_type
        self.is_array = is_array
        self.name = name
        self.value = value

    def __repr__(self):
        s = self.type
        if self.is_array:
            s += '[]'
        s += ' '
        s += self.name
        if self.value:
            s += '='
            s += self.value
        return s


class InterfaceSection:
    def __init__(self):
        self.contents = []
        self.fields = []

    def add_line(self, line):
        stripped = line.strip()
        if not stripped:
            self.contents.append(line)
            return
        elif stripped[0] == '#':
            self.contents.append(line + '\n')
            return
        m = FIELD_LINE.match(line)
        if m:
            field_type, is_array, name, value, comment = m.groups()
            field = InterfaceField(field_type, is_array, name, value)
            self.contents.append(field)
            self.fields.append(field)
            if comment:
                self.contents.append(comment + '\n')
            else:
                self.contents.append('\n')
        else:
            raise Exception('Unable to parse interface line: ' + repr(line))

    def __repr__(self):
        return ''.join(map(str, self.contents))


class ROSInterface(PackageFile):
    def __init__(self, full_path, package):
        super().__init__(full_path, package)

        self.type = full_path.suffix[1:]  # Just the extension, no dot
        self.name = full_path.stem
        self.fn = full_path.name
        self.sections = [InterfaceSection()]

        with open(full_path) as f:
            self.contents = f.read()

        for line in self.contents.split('\n'):
            if AT_LEAST_THREE_DASHES.match(line):
                self.sections.append(InterfaceSection())
                continue
            else:
                self.sections[-1].add_line(line)

    @classmethod
    def is_type(cls, path):
        return cls.suffix == path.suffix

    def get_dependencies(self, dependency_type):
        deps = set()
        if dependency_type != DependencyType.BUILD:
            return deps

        for section in self.sections:
            for field in section.fields:
                if '/' not in field.type:
                    continue
                package = field.type.split('/')[0]
                deps.add(package)

        return deps

    def output(self):
        return '---\n'.join(map(str, self.sections))

    def write(self, output_path):
        with open(output_path, 'w') as f:
            f.write(self.output())

    def __repr__(self):
        return self.name


@package_file
class ROSMsg(ROSInterface):
    suffix = '.msg'

    @classmethod
    def category_name(cls):
        return 'Messages'


@package_file
class ROSSrv(ROSInterface):
    suffix = '.srv'

    @classmethod
    def category_name(cls):
        return 'Services'


@package_file
class ROSAction(ROSInterface):
    suffix = '.action'

    @classmethod
    def category_name(cls):
        return 'Actions'

    def get_dependencies(self, dependency_type):
        deps = super().get_dependencies(dependency_type)
        if dependency_type == DependencyType.BUILD:
            deps.add('actionlib_msgs')
        return deps
