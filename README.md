# ros_introspect

 `ros_introspect` is a Pure Python library for working with the files in ROS packages in a structured way.
It is compatible with both ROS 1 and ROS 2.

You can specify the path directly.

```
from ros_introspect import Package
package = Package('/full/path/geometry_msgs')
```
or you can use the built-in crawler to search recursively

```
from ros_introspect import find_packages

for package in find_packages('/home/dlu/ros2_ws/src'):
    print(package)
```


## The `Package` Class

A `Package` is a path where the `$PATH/package.xml` exists and a collection of `PackageFile`s that are contained within that folder.

Each of the package files are a subtype of `PackageFile` based on their functionality, with each of the subtypes being defined in `ros_introspect.components`, including...

 * `package.xml`
 * `CMakeLists.txt`
 * Source code (Python or C++)
 * ROS Interfaces, i.e. messages, services and actions
 * DynamicReconfig configurations (ROS 1 only)
 * Plugin XML, for use with `pluginlib`
 * Launch files, both `XML` (ROS 1) and Python (ROS 2)
 * Parameter files (`.yaml`)
 * RViz configurations (`.rviz`)
 * Robot model files (`.urdf` and `.xacro`)
 * `setup.py` and `setup.cfg` for Python code
 * Documentation (Markdown, RST, licenses, etc.)
 * Miscellaneous Config files (various metadata like the `.github` folder)

Everything that does not fall into one of those categories ends up as "Other"**

**Excluding `.pyc`, `*~`, and `.git/` files

### PackageXML
The manifest is
 * a parsed XML dom
 * a saved xml header/declaration
 * an optional version number
 * our guess of the standard tab size

### CMakeLists.txt
This file is parsed into a series of Commands, CommandGroups and whitespace/comment strings. CommandGroups are mini CMake objects (groups of commands) surrounded by a pair of matching tags, like `if/endif` or `foreach/endforeach`.

Commands have the form `command_name(sections*)`. Commands track their initial string representation to avoid needless formatting changes. Each Section is an optional initial section_name, followed by some number of tokens. Each Section also has a defined SectionStyle.

### Source Code
The source code is a collection of individual source code files. Each file has a language variable as well as a set of tags. Right now, possible tags include
 * `library` - C++ library file
 * `executable` - C++ executable file
 * `test` - Used only in tests

`setup.py` is not considered a source file.

### Generator Files (Messages, services and actions)
These files are parsed to determine their package dependencies.

### Launch Files
Launch files are parsed XML and are read-only. There is a flag for determining whether the launch file is for tests.

### Dynamic Reconfigure Configs
These files are not parsed, and only their filename is stored.

### Plugin Configurations
This XML configuration can be read and written depending on the needs of the `pluginlib` macro invocations.
