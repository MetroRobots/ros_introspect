from ..package import PackageFile, package_file
import collections
import re

VARIABLE_PATTERN = re.compile(r'\$\{([^\}]+)\}')
QUOTED_PATTERN = re.compile(r'"([^"]+)"')


class SectionStyle:
    def __init__(self, prename='', name_val_sep=' ', val_sep=' '):
        self.prename = prename
        self.name_val_sep = name_val_sep
        self.val_sep = val_sep

    def __repr__(self):
        return f'SectionStyle({repr(self.prename)}, {repr(self.name_val_sep)}, {repr(self.val_sep)})'


class Section:
    def __init__(self, name='', values=None, style=None):
        self.name = name
        if values is None:
            self.values = []
        else:
            self.values = list(values)
        if style:
            self.style = style
        else:
            self.style = SectionStyle()

    def add(self, v):
        self.values.append(v)

    def add_values(self, new_values, alpha_order=True):
        """Add the new_values to the values.

        If alpha_order is true AND the existing values are already alphabetized,
        add the new values in alphabetical order.
        """
        # Check if existing values are sorted
        if alpha_order and self.values == sorted(self.values):
            all_values = self.values + list(new_values)
            self.values = sorted(all_values)
        else:
            self.values += sorted(new_values)

    def is_valid(self):
        return len(self.name) > 0 or len(self.values) > 0

    def __repr__(self):
        s = self.style.prename
        if len(self.name) > 0:
            s += self.name
            if len(self.values) > 0 or '\n' in self.style.name_val_sep:
                s += self.style.name_val_sep
        s += self.style.val_sep.join(self.values)
        return s


class Command:
    def __init__(self, command_name):
        self.command_name = command_name
        self.original = None
        self.changed = False
        self.pre_paren = ''
        self.sections = []

    def get_real_sections(self):
        return [s for s in self.sections if not isinstance(s, str)]

    def get_section(self, key):
        for s in self.get_real_sections():
            if s.name == key:
                return s
        return None

    def get_sections(self, key):
        return [s for s in self.get_real_sections() if s.name == key]

    def add_section(self, key, values=None, style=None):
        self.sections.append(Section(key, values, style))
        self.changed = True

    def add(self, section):
        if section:
            self.sections.append(section)
            self.changed = True

    def first_token(self):
        return self.get_real_sections()[0].values[0]

    def remove_sections(self, key):
        bad_sections = self.get_sections(key)
        if not bad_sections:
            return
        self.changed = True
        self.sections = [section for section in self.sections if section not in bad_sections]
        if len(self.sections) == 1 and isinstance(self.sections[0], str):
            self.sections = []

    def get_tokens(self, include_name=False):
        tokens = []
        for section in self.get_real_sections():
            if include_name and section.name:
                tokens.append(section.name)
            tokens += section.values
        return tokens

    def add_token(self, s):
        sections = self.get_real_sections()
        if len(sections) == 0:
            self.add(Section(values=[s]))
        else:
            last = sections[-1]
            last.values.append(s)
        self.changed = True

    def __repr__(self):
        if self.original and not self.changed:
            return self.original

        s = self.command_name + self.pre_paren + '('
        for section in map(str, self.sections):
            if s[-1] not in '( \n' and section[0] not in ' \n':
                s += ' '
            s += section
        if '\n' in s and s[-1] != '\n':
            s += '\n'
        s += ')'
        return s


class CommandGroup:
    def __init__(self, initial_tag, sub, close_tag):
        self.initial_tag = initial_tag
        self.sub = sub
        self.close_tag = close_tag

    def __repr__(self):
        return str(self.initial_tag) + str(self.sub) + str(self.close_tag)


class CMakeSection:
    def __init__(self, file_path=None, initial_contents=None, depth=0):
        self.file_path = file_path
        if initial_contents is None:
            self.contents = []
        else:
            self.contents = initial_contents
        self.content_map = collections.defaultdict(list)
        for content in self.contents:
            if content.__class__ == Command:
                self.content_map[content.command_name].append(content)
            elif content.__class__ == CommandGroup:
                self.content_map['group'].append(content)
        self.depth = depth

        self.variables = {}
        for cmd in self.content_map['set']:
            tokens = cmd.get_tokens(include_name=True)
            self.variables[tokens[0]] = ' '.join(tokens[1:])
        self.variables['PROJECT_NAME'] = self.get_project_name()

    def get_project_name(self):
        project_tags = self.content_map['project']
        if not project_tags:
            return ''
        # Get all tokens just in case the name is all caps
        return project_tags[0].get_tokens(include_name=True)[0]

    def resolve_variables(self, var):
        if isinstance(var, str):
            s = var
            m = VARIABLE_PATTERN.search(s)
            if not m:
                return s

            for k, v in self.variables.items():
                s = s.replace('${%s}' % k, v)
            return s
        else:
            tokens = []
            for token in var:
                if token and token[0] == '#':
                    continue
                m = QUOTED_PATTERN.match(token)
                if m:
                    token = m.group(1)
                token = self.resolve_variables(token)
                tokens += token.split(' ')
            return tokens

    def get_resolved_tokens(self, cmd, include_name=False):
        return self.resolve_variables(cmd.get_tokens(include_name))


@package_file
class CMake(PackageFile):
    def __init__(self, full_path, package):
        super().__init__(full_path, package)

        from .cmake_parser import parse_file
        self.contents = parse_file(self.full_path)

    @classmethod
    def is_type(cls, path):
        return path.name == 'CMakeLists.txt'

    def write(self, output_path):
        with open(output_path, 'w') as f:
            f.write(str(self.contents))
