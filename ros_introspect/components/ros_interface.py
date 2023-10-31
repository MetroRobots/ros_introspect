from ..package import PackageFile, package_file
import re

AT_LEAST_THREE_DASHES = re.compile(r'^\-{3,}\r?$')
FIELD_LINE = re.compile(r'([\w_/]+)(\[\d*\])?\s+([\w_]+)\s*?(=.*)?(\s*\#.*)?$', re.DOTALL)
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
    def __init__(self, full_path, package_root):
        super().__init__(full_path, package_root)

        self.type = full_path.suffix[1:]  # Just the extension, no dot
        self.name = full_path.stem
        self.sections = [InterfaceSection()]

        self.dependencies = set()

        with open(full_path) as f:
            self.contents = f.read()

        for line in self.contents.split('\n'):
            if AT_LEAST_THREE_DASHES.match(line):
                self.sections.append(InterfaceSection())
                continue
            else:
                self.sections[-1].add_line(line)

        for section in self.sections:
            for field in section.fields:
                if '/' not in field.type:
                    continue
                package, part = field.type.split('/')
                if package != self.name:
                    self.dependencies.add(package)

        if self.type == 'action':
            self.dependencies.add('actionlib_msgs')

    @classmethod
    def is_type(cls, path):
        return cls.suffix == path.suffix

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
