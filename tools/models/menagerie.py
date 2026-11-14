# SPDX-License-Identifier: Apache-2.0
"""Fetch and verify the pinned MuJoCo Menagerie UR5e model.

    menagerie.py fetch <commit> <dest>    blobless sparse fetch of one commit
    menagerie.py verify <dir> <sums>      exit 0 verified, 1 mismatch, 2 usage,
                                          3 cannot verify (I/O error, malformed sums file)
"""
import hashlib
import pathlib
import re
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


# sha256sum text format: 64 lowercase hex digits, two spaces, a relative path.
SUMS_LINE = re.compile(r"([0-9a-f]{64})  (\S.*)")


class CannotVerify(Exception):
    """The tree could not be compared with the sums: unreadable input or a malformed sums file.
    Distinct from a mismatch, which is a finding about the tree."""


def _read_sums(sums_file: pathlib.Path) -> dict[str, str]:
    expected = {}
    for number, line in enumerate(sums_file.read_text().splitlines(), start=1):
        if not line.strip():
            continue
        match = SUMS_LINE.fullmatch(line)
        if match is None:
            raise CannotVerify(f"{sums_file}:{number}: malformed line {line!r}")
        digest, rel = match.groups()
        if rel in expected:
            raise CannotVerify(f"{sums_file}:{number}: duplicate entry for {rel}")
        expected[rel] = digest
    return expected


def verify(model_dir: pathlib.Path, sums_file: pathlib.Path) -> list[str]:
    """Return one line per problem; an empty list means the tree matches the sums. Raises
    CannotVerify or OSError when the comparison cannot be made."""
    model_dir = pathlib.Path(model_dir)
    expected = _read_sums(pathlib.Path(sums_file))
    if not model_dir.is_dir():
        raise CannotVerify(f"{model_dir} is not a directory")
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
        try:
            problems = verify(pathlib.Path(argv[2]), pathlib.Path(argv[3]))
        except (OSError, CannotVerify) as e:
            print(f"menagerie.py: cannot verify: {e}", file=sys.stderr)
            return 3
        for line in problems:
            print(line, file=sys.stderr)
        return 1 if problems else 0
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
