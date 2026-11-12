"""Static style check required by assignment 21.1 (ament_flake8).

Every Python file of the package is checked; nothing is excluded. Until the
midterm some teammate files were exempt and tracked by an expected-failure
test; that style debt has been paid (midterm review), so the exemption list
and the expected failure are gone.
"""
from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, 'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
