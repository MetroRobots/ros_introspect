from ..package import PackageFile, package_file, DependencyType
import ruamel.yaml


rviz_yaml = ruamel.yaml.YAML()
rviz_yaml.indent(mapping=2, sequence=4, offset=2)
rviz_yaml.width = 4096
rviz_yaml.representer.add_representer(type(None),
                                      lambda self, data:
                                      self.represent_scalar('tag:yaml.org,2002:null', '~')
                                      )


def get_class_dicts(entry):
    classes = []
    if isinstance(entry, list):
        for sub in entry:
            classes += get_class_dicts(sub)
    elif isinstance(entry, dict):
        if entry.get('Class'):
            classes.append(entry)
        for k, v in entry.items():
            classes += get_class_dicts(v)
    return classes


def dictionary_subtract(alpha, beta):
    changed = False
    for k in beta.keys():
        if k not in alpha:
            continue
        v = alpha[k]
        if isinstance(v, dict):
            changed |= dictionary_subtract(v, beta[k])
            if not v:
                del alpha[k]
                changed = True
        elif v == beta[k]:
            del alpha[k]
            changed = True
        elif isinstance(v, float):
            delta = abs(beta[k] - v)
            if delta < 1e-4:
                del alpha[k]
                changed = True
            else:
                alpha[k] = round(v, 7)
                changed = True
    return changed


@package_file
class RVizConfig(PackageFile):
    def __init__(self, full_path, package):
        super().__init__(full_path, package)
        if full_path.exists():
            self.contents = rviz_yaml.load(open(full_path))
        else:
            self.contents = None

    @classmethod
    def is_type(cls, path):
        return path.suffix == '.rviz'

    @classmethod
    def needs_share_installation(cls):
        return True

    @classmethod
    def attribute_name(cls):
        return 'rviz_config'

    def get_class_dicts(self):
        return get_class_dicts(self.contents)

    def get_dependencies(self, dependency_type):
        deps = set()
        if dependency_type != DependencyType.RUN:
            return deps
        for config in self.get_class_dicts():
            value = config['Class'].split('/')[0]
            deps.add(value)
        return deps

    def write(self, output_path):
        with open(output_path, 'w') as f:
            rviz_yaml.dump(self.contents, f)
