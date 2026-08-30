"""Dedicated CadQuery/OCP STEP-to-STL process entry."""

import os
import sys


def _run_worker(input_path, output_path):
    try:
        import cadquery
    except Exception:
        return 2
    try:
        imported = cadquery.importers.importStep(input_path, unit="MM")
        shapes = imported.vals()
        if not shapes:
            return 3
    except Exception:
        return 3
    try:
        converted = cadquery.Compound.makeCompound(shapes).exportStl(
            output_path, tolerance=0.1, angularTolerance=0.1, ascii=False,
            relative=False, parallel=False)
        return 0 if converted else 4
    except Exception:
        return 4


def _main(arguments):
    if len(arguments) != 2:
        return 64
    return _run_worker(arguments[0], arguments[1])


if __name__ == "__main__":
    status = _main(sys.argv[1:])
    # CadQuery 2.8.0/OCP can fault during Windows interpreter finalization after
    # synchronous STL export; normal SystemExit therefore corrupts worker status.
    os._exit(status)
