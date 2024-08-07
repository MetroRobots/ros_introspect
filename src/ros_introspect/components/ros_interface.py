from ..package import PackageTextFile, package_file, DependencyType
import re

AT_LEAST_THREE_DASHES = re.compile(r'^\-{3,}\s*$')
FIELD_LINE = re.compile(r'\s*'  # Leading Whitespace
                        r'([\w_/]+|string<=\d+)'  # Package/primitive name (or funky string def)
                        r'(\[<?=?\d*\])?'  # Optional array size
                        r'\s+'  # Some whitespace
                        r'([\w_]+)'  # Field name
                        r'\s*?'  # Optional whitespace
                        r'(?:(=|\s)\s*([^\s\#][^\#]*))?'  # Optional equals sign and default/constant value
                        r'(\s*\#.*)?$', re.DOTALL)  # Trailing whitespace and comment
PRIMITIVES = ['bool', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64',
              'float32', 'float64', 'string', 'time', 'duration']


class InterfaceField:
    def __init__(self, field_type, array_def, name, constant_value=None, default_value=None):
        self.type = field_type
        self.array_def = array_def
        self.name = name
        self.constant_value = constant_value
        self.default_value = default_value

    def __repr__(self):
        s = self.type
        if self.array_def:
            s += self.array_def
        s += ' '
        s += self.name
        if self.constant_value is not None:
            s += '='
            s += self.constant_value
        if self.default_value is not None:
            s += ' '
            s += self.default_value
        return s


class InterfaceSection:
    def __init__(self, separator_line=None):
        self.contents = []
        self.fields = []
        self.separator_line = separator_line

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
            field_type, array_def, name, maybe_equals, value, comment = m.groups()
            if maybe_equals == '=':
                field = InterfaceField(field_type, array_def, name, constant_value=value)
            elif maybe_equals == ' ':
                field = InterfaceField(field_type, array_def, name, default_value=value)
            else:
                field = InterfaceField(field_type, array_def, name)
            self.contents.append(field)
            self.fields.append(field)
            if comment:
                self.contents.append(comment + '\n')
            else:
                self.contents.append('\n')
        else:
            raise Exception('Unable to parse interface line: ' + repr(line))  # pragma: no cover

    def __repr__(self):
        s = ''.join(map(str, self.contents))
        if self.separator_line:
            return self.separator_line + '\n' + s
        else:
            return s


class ROSInterface(PackageTextFile):
    def __init__(self, full_path, package):
        super().__init__(full_path, package)

        self.type = full_path.suffix[1:]  # Just the extension, no dot
        self.name = full_path.stem
        self.fn = full_path.name
        self.sections = [InterfaceSection()]

        for line in self.get_lines():
            if AT_LEAST_THREE_DASHES.match(line):
                self.sections.append(InterfaceSection(line))
                continue
            else:
                self.sections[-1].add_line(line)

    @classmethod
    def is_type(cls, path):
        return cls.suffix == path.suffix

    def get_dependencies(self, dependency_type):
        deps = set()

        if self.package.ros_version == 1:
            if dependency_type == DependencyType.BUILD:
                deps.add('message_generation')
            elif dependency_type == DependencyType.RUN:
                deps.add('message_runtime')
        else:
            if dependency_type == DependencyType.BUILD:
                deps.add('rosidl_default_generators')
            elif dependency_type == DependencyType.RUN:
                deps.add('rosidl_default_runtime')

        if dependency_type == DependencyType.BUILD:
            deps.update(self.get_message_deps())

        return deps

    def get_message_deps(self):
        deps = set()
        for section in self.sections:
            for field in section.fields:
                if '/' not in field.type:
                    continue
                package = field.type.split('/')[0]
                if package == self.package.name:
                    continue
                deps.add(package)

        return deps

    def regenerate_contents(self):
        return ''.join(map(str, self.sections))


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

    def get_message_deps(self):
        deps = super().get_message_deps()
        deps.add('actionlib_msgs')
        return deps
