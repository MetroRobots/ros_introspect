import datetime
import pathlib
import re
import requests
import yaml

CACHE_FOLDER = pathlib.Path('~/.ros').expanduser() / 'ros_introspect'


def get_download_data(url, filename, data_formatter=None, cache_age_days=3):
    now = datetime.datetime.now()
    cache_filepath = CACHE_FOLDER / filename

    if cache_filepath.exists():
        data = yaml.safe_load(open(cache_filepath))

        if 'last_download' in data and now - data['last_download'] < datetime.timedelta(days=cache_age_days):
            # Data is sufficiently new
            return data

    # Attempt a download
    try:
        req = requests.get(url)
        if req.status_code != 200:
            raise RuntimeError('Non-OK Status')
    except requests.exceptions.ConnectionError:
        print(f'Cannot retrieve {filename} from {url}')
        return

    if data_formatter is None:
        if url.endswith('.yaml'):
            data = yaml.safe_load(req.text)
        else:
            # Assume json
            data = req.json()
    else:
        data = data_formatter(req)

    # Save download date
    data['last_download'] = now

    CACHE_FOLDER.mkdir(exist_ok=True, parents=True)
    yaml.dump(data, open(cache_filepath, 'w'))
    return data


def convert_to_underscore_notation(name):
    # https://stackoverflow.com/questions/1175208/elegant-python-function-to-convert-camelcase-to-snake-case
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()


def convert_to_caps_notation(name):
    return ''.join([x.title() for x in name.split('_')])
