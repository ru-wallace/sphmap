#!/usr/bin/env bash

set -ex

prettier -c www
zig fmt --check src
zig build
pushd res/gtfs/; zip ../../reduced_transit.zip *; popd;
./zig-out/bin/make_site --input-www www --output output --index-wasm ./zig-out/bin/index.wasm --osm-data ./res/planet_-123.114,49.284_-123.107,49.287.osm --gtfs-data reduced_transit.zip

valgrind --suppressions=suppressions.valgrind --leak-check=full --track-fds=yes --error-exitcode=1 ./zig-out/bin/sphmap_nogui output/map_data.bin output/map_data.json
valgrind --suppressions=suppressions.valgrind --leak-check=full --track-fds=yes --error-exitcode=1 ./zig-out/bin/pp_benchmark --map-data-bin output/map_data.bin  --map-data-json output/map_data.json --start-id 77 --end-id 141 --attribute-cost highway footway 0.8 --attribute-cost highway secondary 1.5 --turning-cost 1.0

