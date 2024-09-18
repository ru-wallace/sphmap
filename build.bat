zig build -freference-trace
::.\zig-out\bin\mapParser.exe  --input-www www --output output --index-wasm ./zig-out/bin/index.wasm --osm-data ./res/map.osm 
copy ".\zig-out\bin\index.wasm" ".\output\index.wasm"
C:\Users\xgb17134\.conda\envs\general\python.exe -m http.server --directory output 