#!/usr/bin/env bash
set -euo pipefail

source_directory="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
build_directory="${AR4_KINEMATICS_BUILD_DIR:-${source_directory}/build}"
python_command="${PYTHON:-python3}"
python_path="$(command -v "${python_command}")"
pybind11_directory="$("${python_path}" -m pybind11 --cmakedir)"

cmake \
    -S "${source_directory}" \
    -B "${build_directory}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DPYBIND11_FINDPYTHON=ON \
    -DPython_EXECUTABLE="${python_path}" \
    -Dpybind11_DIR="${pybind11_directory}"
cmake --build "${build_directory}" --parallel
