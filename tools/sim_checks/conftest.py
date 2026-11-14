# SPDX-License-Identifier: Apache-2.0
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).parent))

from sim import Sim, SimError  # noqa: E402


@pytest.fixture(scope="module")
def sim(request):
    """One headless simulator per test module. A start that fails is a test failure, never a
    skip; the finalizer stops it and asserts its process group is gone."""
    s = Sim(request.module.__name__)
    request.addfinalizer(s.stop)
    try:
        s.start()
    except SimError as e:
        reason = s.failure_report(e)
    else:
        return s
    pytest.fail(reason)
