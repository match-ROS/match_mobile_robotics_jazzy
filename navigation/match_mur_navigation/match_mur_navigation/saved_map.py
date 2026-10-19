"""Resolve and validate repository maps and their coordinate metadata."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory


def default_map():
    return Path(get_package_share_directory('match_mur_navigation')) / 'maps' / 'mur620c_mocap.yaml'


def map_metadata(path):
    path = Path(path).expanduser().resolve()
    with path.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not data.get('image') or float(data.get('resolution', 0)) <= 0:
        raise ValueError('Ungültige Karten-YAML')
    if not (path.parent / data['image']).is_file():
        raise ValueError('Das Kartenbild fehlt')
    metadata = path.with_suffix('.mapping.yaml')
    if not metadata.is_file():
        raise ValueError('Kartenbezug fehlt: zugehörige .mapping.yaml erforderlich')
    with metadata.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or document.get('frame') != 'mocap':
        raise ValueError('Der Mocap-Vergleich benötigt eine Karte im Frame mocap')
    return path, document
