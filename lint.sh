#!/usr/bin/env bash

rm ./make_site.log
set -ex

#prettier -c www
zig fmt --check src
zig build -freference-trace
./zig-out/bin/make_site --input-www www --output output --index-wasm ./zig-out/bin/index.wasm --osm-data ./res/map.osm
python -m http.server --directory output 