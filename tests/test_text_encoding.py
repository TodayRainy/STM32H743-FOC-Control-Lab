from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[1]
MANAGED_ROOTS = (
    "algorithm",
    "app",
    "Core",
    "docs",
    "driver",
    "host",
    "mdk",
    "port",
    "system",
    "tests",
)
TEXT_SUFFIXES = {
    ".c",
    ".cpp",
    ".h",
    ".hpp",
    ".json",
    ".md",
    ".ps1",
    ".py",
    ".txt",
    ".uvprojx",
    ".yaml",
    ".yml",
}
def managed_text_files():
    result = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
    )
    root_files = {
        ".editorconfig", "AGENTS.md", "CHANGELOG.md", "README.md",
        "requirements-host.txt",
    }

    for raw_name in result.stdout.split(b"\0"):
        if not raw_name:
            continue
        relative = Path(raw_name.decode("utf-8"))
        path = REPO_ROOT / relative
        if not path.is_file() or path.suffix.lower() not in TEXT_SUFFIXES:
            continue
        if relative.parts[0] in MANAGED_ROOTS or relative.as_posix() in root_files:
            yield path


def test_first_party_text_is_utf8_without_bom():
    invalid = []
    with_bom = []

    for path in managed_text_files():
        data = path.read_bytes()
        if data.startswith(b"\xef\xbb\xbf"):
            with_bom.append(str(path.relative_to(REPO_ROOT)))
        try:
            data.decode("utf-8", errors="strict")
        except UnicodeDecodeError:
            invalid.append(str(path.relative_to(REPO_ROOT)))

    assert invalid == [], f"non-UTF-8 first-party text: {invalid}"
    assert with_bom == [], f"UTF-8 BOM is not used: {with_bom}"
