#!/bin/sh
set -eu
cd "$(dirname "$0")"
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/stac-d4.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM
"${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror -pedantic -I. latch_test.cpp -o "$build_dir/latch_test"
"$build_dir/latch_test"
