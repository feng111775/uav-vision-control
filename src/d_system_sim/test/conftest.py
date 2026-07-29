"""Make the package importable when pytest is run from the workspace root."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))
