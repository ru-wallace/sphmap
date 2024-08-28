#!/usr/bin/env bash

set -ex

#prettier -c www
zig fmt --check src
zig build
rm ./make_site.log

./zig-out/bin/make_site --input-www www --output output --index-wasm ./zig-out/bin/index.wasm --osm-data ./res/map.osm
