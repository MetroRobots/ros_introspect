from ..package import SingularPackageFile, PackageTextFile, package_file
import ast
import collections
import pathlib
import re
import sys

# Version-Dependent AST operations
if sys.version_info.major == 3 and sys.version_info.minor >= 8:
    def is_constant(el):
        return isinstance(el.value, ast.Constant)

    get_source_segment = ast.get_source_segment
else:  # pragma: no cover
    def is_constant(el):
        return isinstance(el.value, ast.Str)

    def get_source_segment(source, node):
        """Get source code segment of the *source* that generated *node*.
        If some location information (`lineno`, `end_lineno`, `col_offset`,
        or `end_col_offset`) is missing, return None.
        """
        try:
            lineno = node.lineno - 1
            end_lineno = node.end_lineno - 1
            col_offset = node.col_offset
            end_col_offset = node.end_col_offset
        except AttributeError:
            return None

        lines = ast._splitlines_no_ff(source)
        if end_lineno == lineno:
            return lines[lineno].encode()[col_offset:end_col_offset].decode()

        first = lines[lineno].encode()[col_offset:].decode()
        last = lines[end_lineno].encode()[:end_col_offset].decode()
        lines = lines[lineno + 1:end_lineno]

        lines.insert(0, first)
        lines.append(last)
        return ''.join(lines)

HELPER_FUNCTIONS = {
    'catkin_pkg.python_setup': 'generate_distutils_setup'
}

IMPORT_TEMPLATE = 'from {} import {}\n'
QUOTE_CHARS = ['"', "'"]
LINE_LENGTH = 100
GLOB_PATTERN = re.compile(r'glob\([\'"](.*)[\'"]\)')


def python_collection_to_lines(obj, indent=4, multi_line=False):
    """Convert a python collection (list/tuple/dict) to a sequence of lines, NOT including brackets

       indent is the size of the indent
       multi_line determines whether lists and tuples should be spread on multiple lines by default a la
            multi_line=False : [1, 2, 3]
            multi_line=True  : [
                                1,
                                2,
                                3,
                               ]
    """
    if isinstance(obj, dict):
        items = []
        for k, v in obj.items():
            s = k + ': '
            sub_lines = python_to_lines(v, len(s) + indent, indent + 4)
            joiner = '\n' + (' ' * 4)
            s += joiner.join(sub_lines)
            items.append(s)
    elif multi_line:
        lines = []
        for item in obj:
            sub_lines = python_to_lines(item, indent, indent + 4)
            s = sub_lines[0]
            for line in sub_lines[1:]:
                s += '\n' + ' ' * (indent + 4) + line
            s += ','
            lines.append(s)
        return lines
    else:
        items = []
        for item in obj:
            items += python_to_lines(item, indent, indent + 4)

    line = ', '.join(items)
    if indent + len(line) >= LINE_LENGTH:
        multi_line = True
    if multi_line:
        return [item for item in items[:-1]] + [items[-1]]

    return [line]


def python_to_lines(obj, initial_length=0, indent=4):
    """Convert a python object to the properly formatted python code.

    initial_length is the length of the string before the object
    indent is the size of the indent that each line should have
    """
    if isinstance(obj, list):
        brackets = '[]'
    elif isinstance(obj, tuple):
        brackets = '()'
    elif isinstance(obj, dict) or isinstance(obj, collections.OrderedDict):
        brackets = '{}'
    else:
        # For other objects, we just return the string representation of the object as a single line
        return [str(obj)]

    # Generate the inner lines, assuming not multi_line
    inner_lines = python_collection_to_lines(obj, indent, multi_line=False)

    # If the inner contents can fit on a single line, return the single line
    if len(inner_lines) == 1 and '\n' not in inner_lines[0]:
        inner_line = inner_lines[0]
        if initial_length + len(inner_line) < LINE_LENGTH:
            return [brackets[0] + inner_line + brackets[1]]

    # Regenerate the inner lines, assuming it IS multi_line
    inner_lines = python_collection_to_lines(obj, indent, multi_line=True)
    # Place the open and closing brackets on their own lines surrounding the inner_lines
    lines = [brackets[0]]
    for line in inner_lines:
        # Indent each line a little more
        line = line.replace('\n', '\n    ')
        lines.append('    ' + line)
    lines.append(brackets[1])
    return lines


def quote_string(s, quote_char="'"):
    """Utility function to wrap an arbitrary string in a quote character"""
    return quote_char + str(s) + quote_char


def unquote_string(s):
    """Utility function to remove the quote chars from a string"""
    if s and s[0] == s[-1] and s[0] in QUOTE_CHARS:
        return s[1:-1]
    return s


def contains_quoted_string(container, s):
    """Utility function to determine if the string is present in the container wrapped in some quote char"""
    for quote_char in QUOTE_CHARS:
        quoted_value = quote_string(s, quote_char)
        if quoted_value in container:
            return quoted_value


@package_file
class SetupPy(SingularPackageFile, PackageTextFile):
    """
    Representation of a setup.py file, covering a large range of different styles

    The core operation is generating the dictionary of arguments sent to the setup function.
    If a helper function (like generate_distutils_setup) is used, there won't be many arguments.

    Key fields:
    hash_bang (bool) - Whether the file starts with a #!/usr/bin/env python
    imports (list of tuples) - Used for generating the import statements like `from X import Y`
                               First element of tuple is a string of the name of the module (e.g. X)
                               Second element is the string(s) representing the function(s) to import (e.g. Y)
    declare_package_name (bool) - Whether to declare the package name as a string for later use
    helper_function (optional string) - Name of module to find the helper function to call (see HELPER_FUNCTIONS)
    helper_variable (optional string) - Name of variable to store the results of the helper function in.
    args (ordered dictionary) - Arguments to pass to setup/helper_function
                                The keys are regular strings that represent the name of the variable.
                                The values are more complex.
    """

    def __init__(self, full_path, package):
        PackageTextFile.__init__(self, full_path, package)

        self.args = collections.OrderedDict()

        if not self.contents:
            self.changed = True
            self.hash_bang = True
            if package.ros_version == 1:
                self.imports = [('distutils.core', 'setup'), ('catkin_pkg.python_setup', 'generate_distutils_setup')]
                self.declare_package_name = False
                self.helper_function = 'catkin_pkg.python_setup'
                self.helper_variable = 'package_info'
                self.args['packages'] = [quote_string(self.package.name)]
                self.args['package_dir'] = {quote_string(''): quote_string('src')}
            else:
                self.imports = [('setuptools', 'setup')]
                self.declare_package_name = True
                self.helper_function = None
                self.helper_variable = None
                self.args['name'] = 'package_name'
            return

        self.hash_bang = (self.contents[0] == '#')
        self.imports = []
        self.declare_package_name = False
        self.helper_function = None
        self.helper_variable = None

        # Split into imports / body
        import_elements = []
        body_elements = []
        for el in ast.parse(self.contents).body:
            if isinstance(el, ast.ImportFrom):
                import_elements.append(el)
            else:
                body_elements.append(el)

        # Examine Imports
        for el in import_elements:
            if el.module in HELPER_FUNCTIONS:
                self.helper_function = el.module
            self.imports.append((el.module, [x.name for x in el.names]))

        def ast_to_python(el):
            """Helper function to convert an ast element to its Python data structure"""
            if isinstance(el, ast.Dict):
                d = {}
                for k, v in zip(el.keys, el.values):
                    d[ast_to_python(k)] = ast_to_python(v)
                return d
            elif isinstance(el, ast.List):
                return [ast_to_python(elt) for elt in el.elts]
            elif isinstance(el, ast.Tuple):
                return tuple(ast_to_python(elt) for elt in el.elts)
            else:
                return get_source_segment(self.contents, el)

        # Determine variable name and dictionary args
        for el in body_elements:
            if isinstance(el, ast.Assign):
                if is_constant(el):
                    self.declare_package_name = True
                else:
                    self.helper_variable = el.targets[0].id

                    for keyword in el.value.keywords:
                        self.args[keyword.arg] = ast_to_python(keyword.value)

            elif isinstance(el, ast.Expr) and self.helper_variable is None:
                for keyword in el.value.keywords:
                    self.args[keyword.arg] = ast_to_python(keyword.value)

    def set_arg(self, key, value):
        f_value = repr(value)
        if self.args.get(key) == f_value:
            return
        self.args[key] = f_value
        self.changed = True

    def get_share_path(self, rel_fn=None):
        if str(rel_fn) == 'resource':
            return "'share/ament_index/resource_index/packages'"

        if self.declare_package_name:
            base_folder = "'share/' + package_name"
        else:
            base_folder = f"'share/{self.package.name}'"

        if rel_fn is None:
            return base_folder

        if self.declare_package_name:
            return base_folder + f" + '/{rel_fn}'"
        else:
            return base_folder + f'/{rel_fn}'

    def include_data_files(self, paths, folder=None):
        if 'data_files' not in self.args:
            self.args['data_files'] = []

        # rewrite tuples as dict
        existing_folders = {d[0]: d[1] for d in self.args['data_files']}

        install_folder = self.get_share_path(folder)
        if install_folder not in existing_folders:
            existing_files = []
            self.args['data_files'].append((install_folder, existing_files))
        else:
            existing_files = existing_folders[install_folder]
            if isinstance(existing_files, str):
                m = GLOB_PATTERN.match(existing_files)
                if m:
                    glob_pattern = m.group(1)
                    existing_files = []
                    root = self.full_path.parent
                    for subpath in root.glob(glob_pattern):
                        existing_files.append(quote_string(str(subpath).replace(str(root) + '/', '')))

            if not isinstance(existing_files, list):  # pragma: no cover
                raise RuntimeError('Trouble understanding the install data_files bit of the setup.py')

        if str(folder) == 'resource' and self.declare_package_name and "'resource/' + package_name" in existing_files:
            paths.remove(self.package.name)

        for fn in paths:
            if folder:
                resolved = quote_string(folder / fn)
            else:
                resolved = quote_string(fn)
            if resolved not in existing_files:
                existing_files.append(resolved)
                self.changed = True

        # If the new command would span more than 1 line, use a glob operation instead
        if len(python_to_lines(existing_files, 8)) > 1:
            index = [k[0] for k in self.args['data_files']].index(install_folder)
            self.args['data_files'][index] = (install_folder, f"glob('{folder}/*')")
            import_item = ('glob', ['glob'])
            if import_item not in self.imports:
                self.imports.append(import_item)

    def regenerate_contents(self):
        s = ''

        if self.hash_bang:
            s += '#!/usr/bin/env python\n\n'

        for module, names in self.imports:
            if isinstance(names, list):
                names = ', '.join(sorted(names))
            s += IMPORT_TEMPLATE.format(module, names)

        s += '\n'

        if self.declare_package_name:
            s += "package_name = '{}'\n\n".format(self.package.name)

        if self.helper_function:
            function_name = HELPER_FUNCTIONS[self.helper_function]
            s += '{} = {}(\n'.format(self.helper_variable, function_name)
            final_piece = ')\n\nsetup(**{})\n'.format(self.helper_variable)
        else:
            s += 'setup(\n'
            final_piece = ')\n'

        for k, v in self.args.items():
            line = '    {}='.format(k)
            lines = python_to_lines(v, len(line))
            s += line + lines[0]
            for line in lines[1:]:
                s += '\n    ' + line
            s += ',\n'

        s += final_piece
        return s


def create_setup_py(package):
    if package.setup_py:
        return
    package.add_file(SetupPy(package.root / 'setup.py', package))


if __name__ == '__main__':  # pragma: no coverage
    output_dir = pathlib.Path('output_data')
    output_dir.mkdir(exist_ok=True)

    rows = []
    for filename in sorted(pathlib.Path('test_data').glob('*py')):
        spy = SetupPy('TEST_PACKAGE_NAME', filename)

        output_file = output_dir / filename.name
        with open(output_file, 'w') as f:
            f.write(str(spy))
