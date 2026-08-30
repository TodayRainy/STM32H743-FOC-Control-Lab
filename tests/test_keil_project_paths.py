from pathlib import Path
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[1]
PROJECT = REPO_ROOT / "mdk" / "STM32H743VGT6_FOC.uvprojx"


def project_path(value: str) -> Path:
    return (PROJECT.parent / value.replace("\\", "/")).resolve()


def test_keil_project_files_and_include_directories_exist():
    root = ET.parse(PROJECT).getroot()
    files = [node.text for node in root.findall(".//FilePath") if node.text]
    include_sections = [
        node.text for node in root.findall(".//IncludePath")
        if node.text and node.text.strip()
    ]
    assert len(include_sections) == 1

    missing_files = [value for value in files if not project_path(value).is_file()]
    include_dirs = include_sections[0].split(";")
    missing_dirs = [
        value for value in include_dirs if not project_path(value).is_dir()
    ]
    assert not missing_files
    assert not missing_dirs
