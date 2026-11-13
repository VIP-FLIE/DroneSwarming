"""Retain uncaught Python and fatal-signal diagnostics directly in a node's log directory."""

from __future__ import annotations

import faulthandler
import os
import runpy
import sys
import traceback
from pathlib import Path


def main() -> None:
    """Run the existing entry point without intercepting or redirecting its process streams."""
    if len(sys.argv) < 2:
        raise SystemExit('A Python node entry point is required')
    sys.argv = sys.argv[1:]
    directory = Path(os.environ['ROS_LOG_DIR'])
    with (directory / 'python-faults.log').open('w', encoding='utf-8') as diagnostic:
        faulthandler.enable(file=diagnostic, all_threads=True)
        try:
            runpy.run_path(sys.argv[0], run_name='__main__')
        except BaseException as error:
            if not isinstance(error, SystemExit) or error.code not in (None, 0):
                traceback.print_exc(file=diagnostic)
                diagnostic.flush()
            raise
        finally:
            faulthandler.disable()


if __name__ == '__main__':
    main()
