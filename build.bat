zig fmt --check src
zig build -freference-trace
python -m http.server --directory output 