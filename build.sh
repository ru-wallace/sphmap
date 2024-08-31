#!/usr/bin/env bash

rm ./make_site.log
set -ex


zig fmt --check src
zig build -freference-trace
python -m http.server --directory output 