# px4_practice_py_pkg/modules/utils.py

import os
import yaml
import xml.etree.ElementTree as ET
import importlib
import datetime
import csv


# =========================
# Path / Config utilities
# =========================

def get_file_dirname(file):
    """Return absolute directory of the given file path."""
    return os.path.dirname(os.path.abspath(file))


def load_config(config_file):
    with open(config_file, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


# =========================
# Behavior Tree XML
# =========================

def parse_behavior_tree(xml_path):
    """Parse a BehaviorTree XML file and return the root element."""
    tree = ET.parse(xml_path)
    return tree.getroot()


# =========================
# Value conversion
# =========================

def convert_value(v):
    """
    Convert string values from XML:
    - "None" -> None
    - "123"  -> int
    - "1.23" -> float
    """
    if v == "None":
        return None

    if isinstance(v, str):
        if v.isdigit() or (v.startswith('-') and v[1:].isdigit()):
            return int(v)
        try:
            return float(v)
        except ValueError:
            pass

    return v


# =========================
# Optional import (plugin)
# =========================

def optional_import(module_path):
    """
    Try importing a module by path.
    - Return None if the module itself does not exist.
    - Raise error if internal dependency is missing.
    """
    if not module_path:
        return None

    try:
        return importlib.import_module(module_path)
    except ModuleNotFoundError as e:
        if e.name == module_path:
            return None
        raise


# =========================
# Result Saver (PPA / BT expansion용)
# =========================

class ResultSaver:
    """
    Minimal ResultSaver used by ppa_bt_constructor.
    Supports:
    - output filename generation
    - CSV / XML saving
    """

    def __init__(self, config, output_dir="output"):
        self.config = config
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        now = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.timestamp = now

    def generate_output_filename(self, extension="xml"):
        base = f"bt_result_{self.timestamp}"
        return os.path.join(self.output_dir, f"{base}.{extension}")

    def save_csv(self, file_path, header, rows):
        with open(file_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(rows)
