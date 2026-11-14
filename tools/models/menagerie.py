# SPDX-License-Identifier: Apache-2.0
"""Fetch and verify the pinned MuJoCo Menagerie UR5e model.

    menagerie.py fetch <commit> <dest>    blobless sparse fetch of one commit
    menagerie.py verify <dir> <sums>      exit 0 verified, 1 mismatch, 2 usage
"""
import hashlib
import pathlib
import shutil
import subprocess
import sys
import tempfile

REPO = "https://github.com/google-deepmind/mujoco_menagerie.git"
FOLDER = "universal_robots_ur5e"


def _sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_sums(sums_file: pathlib.Path) -> dict[str, str]:
    expected = {}
    for line in sums_file.read_text().splitlines():
        if line.strip():
            digest, _, rel = line.partition("  ")
            expected[rel.strip()] = digest.strip()
    return expected


def verify(model_dir: pathlib.Path, sums_file: pathlib.Path) -> list[str]:
    """Return one line per problem; an empty list means the tree matches the sums."""
    model_dir = pathlib.Path(model_dir)
    expected = _read_sums(pathlib.Path(sums_file))
    actual = {
        p.relative_to(model_dir).as_posix(): p
        for p in sorted(model_dir.rglob("*"))
        if p.is_file()
    }
    problems = []
    for rel, digest in sorted(expected.items()):
        if rel not in actual:
            problems.append(f"missing: {rel}")
        elif _sha256(actual[rel]) != digest:
            problems.append(f"mismatch: {rel}")
    for rel in sorted(set(actual) - set(expected)):
        problems.append(f"unexpected: {rel}")
    return problems


def fetch(commit: str, dest: pathlib.Path) -> None:
    """Fetch exactly one commit of the Menagerie UR5e folder into dest."""
    with tempfile.TemporaryDirectory() as tmp:
        git = ["git", "-C", tmp]
        subprocess.run(["git", "init", "-q", tmp], check=True)
        subprocess.run([*git, "remote", "add", "origin", REPO], check=True)
        subprocess.run([*git, "sparse-checkout", "set", FOLDER], check=True)
        subprocess.run(
            [*git, "fetch", "-q", "--depth", "1", "--filter=blob:none", "origin", commit],
            check=True,
        )
        subprocess.run([*git, "checkout", "-q", "FETCH_HEAD"], check=True)
        shutil.copytree(pathlib.Path(tmp) / FOLDER, dest)


def main(argv: list[str]) -> int:
    if len(argv) == 4 and argv[1] == "fetch":
        fetch(argv[2], pathlib.Path(argv[3]))
        return 0
    if len(argv) == 4 and argv[1] == "verify":
        problems = verify(pathlib.Path(argv[2]), pathlib.Path(argv[3]))
        for line in problems:
            print(line, file=sys.stderr)
        return 1 if problems else 0
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
