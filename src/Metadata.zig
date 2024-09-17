const std = @import("std");

min_lat: f32 = std.math.floatMax(f32),
max_lat: f32 = -std.math.floatMax(f32),
min_lon: f32 = std.math.floatMax(f32),
max_lon: f32 = -std.math.floatMax(f32),
n_nodes: u64 = 0,
n_foot: u64 = 0,
n_bicycle: u64 = 0,
n_car: u64 = 0,
n_rail: u64 = 0,
n_other: u64 = 0,
n_buildings: u64 = 0,
