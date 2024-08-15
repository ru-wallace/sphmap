const std = @import("std");
const map_data = @import("map_data.zig");
const PointLookup = map_data.PointLookup;
const NodeId = map_data.NodeId;
const Allocator = std.mem.Allocator;
const lin = @import("lin.zig");
const Point = lin.Point;
const Vec = lin.Vec;

const ClosestStopLookup = @This();

pub const x_samples: usize = 200;
pub const y_samples: usize = 150;
pub const sentinel = Vec{ .x = std.math.inf(f32), .y = std.math.inf(f32) };

alloc: Allocator,
width: f32,
height: f32,
stop_list: []const NodeId,
point_lookup: *const PointLookup,
storage: []Vec,

pub fn init(alloc: Allocator, stop_list: []const NodeId, point_lookup: *const PointLookup, width: f32, height: f32) !ClosestStopLookup {
    const storage = try alloc.alloc(Vec, x_samples * y_samples);
    errdefer alloc.free(storage);

    var ret = ClosestStopLookup{
        .alloc = alloc,
        .width = width,
        .height = height,
        .stop_list = &.{},
        .point_lookup = point_lookup,
        .storage = storage,
    };

    try ret.updateStops(stop_list);

    return ret;
}

pub fn deinit(self: *ClosestStopLookup) void {
    self.alloc.free(self.stop_list);
    self.alloc.free(self.storage);
}

pub fn updateStops(self: *ClosestStopLookup, stop_list: []const NodeId) !void {
    const new_stop_list = try self.alloc.dupe(NodeId, stop_list);
    errdefer self.alloc.free(new_stop_list);

    self.alloc.free(self.stop_list);
    self.stop_list = new_stop_list;

    @memset(self.storage, sentinel);
}

fn updateSample(self: *const ClosestStopLookup, x: usize, y: usize) Vec {
    const x_m = idxToPos(x, x_samples, self.width);
    const y_m = idxToPos(y, y_samples, self.height);
    const sample_pos: lin.Point = .{
        .x = x_m,
        .y = y_m,
    };

    var closest_dist = std.math.inf(f32);
    var closest_vec: Vec = undefined;

    std.debug.assert(self.stop_list.len > 0);

    for (self.stop_list) |stop| {
        const stop_pos = self.point_lookup.get(stop);
        const v = stop_pos.sub(sample_pos);
        const dist = v.length_2();
        if (closest_dist > dist) {
            closest_dist = dist;
            closest_vec = v;
        }
    }

    self.storage[y * x_samples + x] = closest_vec;
    return closest_vec;
}

fn getOffs(self: *const ClosestStopLookup, x: usize, y: usize, pos: Point) f32 {
    const sample_pos = Point{
        .x = @as(f32, @floatFromInt(x)) / (x_samples - 1) * self.width,
        .y = @as(f32, @floatFromInt(y)) / (y_samples - 1) * self.height,
    };
    const additional_offs = sample_pos.sub(pos);
    var sample_offs = self.storage[y * x_samples + x];
    if (std.meta.eql(sample_offs, sentinel)) {
        sample_offs = self.updateSample(x, y);
    }
    const ret = @abs(sample_offs.length() + additional_offs.dot(sample_offs.normalized()));
    return ret;
}

// FIXME: unit test to make sure it lines up with the visualization
pub fn getDistance(self: *const ClosestStopLookup, pos: Point) f32 {
    const sample_space_x = (pos.x / self.width) * (x_samples - 1);
    const sample_space_y = (pos.y / self.height) * (y_samples - 1);
    const sample_space_top = @ceil(sample_space_y);
    const sample_space_bottom = @floor(sample_space_y);
    const sample_space_right = @ceil(sample_space_x);
    const sample_space_left = @floor(sample_space_x);

    const x_lerp = sample_space_x - sample_space_left;
    const y_lerp = sample_space_y - sample_space_bottom;

    const tr = self.getOffs(@intFromFloat(sample_space_right), @intFromFloat(sample_space_top), pos);
    const bl = self.getOffs(@intFromFloat(sample_space_left), @intFromFloat(sample_space_bottom), pos);
    const br = self.getOffs(@intFromFloat(sample_space_right), @intFromFloat(sample_space_bottom), pos);
    const tl = self.getOffs(@intFromFloat(sample_space_left), @intFromFloat(sample_space_top), pos);

    const a = tr * x_lerp + tl * (1.0 - x_lerp);
    const b = br * x_lerp + bl * (1.0 - x_lerp);
    return b * (1.0 - y_lerp) + a * y_lerp;
}

pub fn sampleIt(self: *const ClosestStopLookup) SampleIt {
    return .{
        .inner = self,
    };
}

fn idxToPos(idx: usize, num_samples: usize, len: f32) f32 {
    const idx_f: f32 = @floatFromInt(idx);
    const num_samples_f: f32 = @floatFromInt(num_samples - 1);
    return idx_f / num_samples_f * len;
}

const SampleIt = struct {
    inner: *const ClosestStopLookup,
    idx: usize = 0,

    const Output = struct {
        x: f32,
        y: f32,
        closest_point: Vec,
    };

    pub fn next(self: *SampleIt) ?Output {
        if (self.idx >= self.inner.storage.len) {
            return null;
        }
        defer self.idx += 1;

        const x_idx = self.idx % x_samples;
        const y_idx = self.idx / x_samples;

        const x = idxToPos(x_idx, x_samples, self.inner.width);
        const y = idxToPos(y_idx, y_samples, self.inner.height);

        return .{
            .x = x,
            .y = y,
            .closest_point = self.inner.storage[self.idx],
        };
    }
};
