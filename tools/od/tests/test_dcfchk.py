# SPDX-License-Identifier: Apache-2.0
"""Lely `dcfchk` (dcf-tools 2.3.5) as an independent lint of the drive EDS.

Which tool checks what, as observed by running `dcfchk -n 1` on each broken copy in defects.py
(dcfchk lints sections, keys, access types, value syntax and ranges, and sub-object counts; it
resolves PDO mappings only far enough to look the mapped entries up, so it never judges a
mapping's direction, mappability or length):

  defect               dcfchk -n 1                                       od.validate
  map_72_bits          misses (exit 0)                                   catches
  unmappable_mapped    misses (exit 0)                                   catches
  read_only_in_rpdo    misses (exit 0)                                   catches
  missing_1018_sub1    catches: "1 missing sub-object(s) for object 0x1018"   catches
  nodeid_suffix_form   catches: "invalid DefaultValue in [1014]: 0x80+$NODEID" catches

Without `-n`, dcfchk 2.3.5 cannot resolve `$NODEID` and exits 1 with a KeyError traceback on
every copy, the clean EDS included, so a run without a node-ID would "catch" everything and
mean nothing. The node-ID is always passed.

The contract values themselves (object set, types, defaults, PDO layouts of D4/D13) are checked
only by test_od.py; dcfchk knows nothing about this project's contract.
"""
import re
import shutil
import subprocess

import pytest

import defects
import od

# Recorded dcfchk behaviour per broken copy: None when dcfchk accepts the copy (exit 0), or a
# fragment of the diagnostic it prints when it rejects it.
DCFCHK = {
    "map_72_bits": None,
    "unmappable_mapped": None,
    "read_only_in_rpdo": None,
    "missing_1018_sub1": "1 missing sub-object(s) for object 0x1018",
    "nodeid_suffix_form": "invalid DefaultValue in [1014]: 0x80+$NODEID",
}


def _dcfchk(path, *args):
    tool = shutil.which("dcfchk")
    # A missing tool is a failure, never a skip: the lint must not silently stop running.
    assert tool is not None, "dcfchk not on PATH (ros-jazzy-lely-core-libraries in the dev image)"
    return subprocess.run([tool, *(args or ("-n", "1")), str(path)], capture_output=True, text=True)


def test_dcfchk_accepts_the_eds():
    result = _dcfchk(defects.EDS)
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stderr == "", result.stderr


def test_dcfchk_without_node_id_cannot_resolve_the_eds():
    # Recorded so that a dcf-tools upgrade that changes this is noticed and re-calibrated.
    result = _dcfchk(defects.EDS, "-p")
    assert result.returncode != 0
    assert "$NODEID not defined" in result.stderr


def test_calibration_covers_every_defect():
    assert sorted(DCFCHK) == sorted(defects.DEFECTS)


@pytest.mark.parametrize("name", sorted(DCFCHK))
def test_dcfchk_behaviour_on_broken_copy(tmp_path, name):
    path = defects.write_broken(name, tmp_path)
    result = _dcfchk(path)
    expected = DCFCHK[name]
    if expected is None:
        assert result.returncode == 0, result.stdout + result.stderr
        # dcfchk misses this defect, so od.validate must catch it.
        errors = od.validate(od.load(path))
        assert any(re.search(defects.MESSAGES[name], e) for e in errors), errors
    else:
        assert result.returncode != 0, result.stdout + result.stderr
        assert expected in result.stderr, result.stderr
