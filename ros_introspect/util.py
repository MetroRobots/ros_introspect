import datetime
import pathlib
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
