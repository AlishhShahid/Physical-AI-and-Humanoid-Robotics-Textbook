import pytest
import sys

if __name__ == "__main__":
    # Run pytest on the 'test' directory
    retcode = pytest.main(["-v", "test"])
    sys.exit(retcode)
