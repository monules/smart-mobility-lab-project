"""Docstring check required by assignment 21.1 (ament_pep257).

D213 (summary on the *second* line) is turned off: PEP 257 allows the summary
on the first line, and every teammate's module (M1-B, M2-B, M3) uses that
form, so one consistent style is kept across the merged package.
"""
from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test', '--add-ignore', 'D213'])
    assert rc == 0, 'Found code style errors / warnings'
