const std = @import("std");
const Metadata = @import("Metadata.zig");
const Allocator = std.mem.Allocator;
const lin = @import("lin.zig");
const MapPos = lin.Point;
const builtin = @import("builtin");
const Point = lin.Point;
pub const WayBuckets = MapBuckets(WayId);

pub const PointPairToWayMap = NodePairMap(WayId);

pub const NodeId = struct {
    value: u32,
};

pub const WayId = struct {
    value: usize,
};

pub const IndexRange = struct {
    start: usize,
    end: usize,
};

pub const Way = struct {
    node_ids: []const NodeId,

    pub fn fromIndexRange(range: IndexRange, index_buf: []const u32) Way {
        return .{
            .node_ids = @ptrCast(index_buf[range.start..range.end]),
        };
    }

    pub fn indexRange(self: *const Way, index_buffer: []const u32) IndexRange {
        const node_ids_ptr: usize = @intFromPtr(self.node_ids.ptr);
        const index_buffer_ptr: usize = @intFromPtr(index_buffer.ptr);
        const start = (node_ids_ptr - index_buffer_ptr) / @sizeOf(u32);
        const end = start + self.node_ids.len;
        return .{
            .start = start,
            .end = end,
        };
    }
};

pub const PointLookup = struct {
    points: []const f32,

    pub fn numPoints(self: *const PointLookup) usize {
        return self.points.len / 2;
    }

    pub fn get(self: *const PointLookup, id: NodeId) MapPos {
        return .{
            .x = self.points[id.value * 2],
            .y = self.points[id.value * 2 + 1],
        };
    }
};

pub const WayLookup = struct {
    ways: []const Way,

    pub const Builder = struct {
        ways: std.ArrayList(Way),
        index_buffer: []const u32,

        pub fn init(alloc: Allocator, index_buffer: []const u32) Builder {
            return .{
                .ways = std.ArrayList(Way).init(alloc),
                .index_buffer = index_buffer,
            };
        }

        pub fn deinit(self: *Builder) void {
            self.ways.deinit();
        }

        pub fn feed(self: *Builder, way: Way) !void {
            try self.ways.append(way);
        }

        pub fn build(self: *Builder) !WayLookup {
            const ways = try self.ways.toOwnedSlice();
            return .{
                .ways = ways,
            };
        }
    };

    pub fn deinit(self: *WayLookup, alloc: Allocator) void {
        alloc.free(self.ways);
    }

    pub fn get(self: *const WayLookup, id: WayId) Way {
        return self.ways[id.value];
    }
};

pub const NodeAdjacencyMap = struct {
    // Where in storage this node's neighbors are. Indexed by NodeId
    segment_starts: []u32,
    // All neighbors for all nodes, Each node's neighbors are contiguous
    storage: []const NodeId,

    pub const Builder = struct {
        arena: *std.heap.ArenaAllocator,
        node_neighbors: []std.AutoArrayHashMapUnmanaged(NodeId, void),

        pub fn init(alloc: Allocator, num_points: usize) !Builder {
            var arena = try alloc.create(std.heap.ArenaAllocator);
            errdefer alloc.destroy(arena);
            arena.* = std.heap.ArenaAllocator.init(alloc);

            const arena_alloc = arena.allocator();
            const node_neighbors = try arena_alloc.alloc(std.AutoArrayHashMapUnmanaged(NodeId, void), num_points);
            @memset(node_neighbors, std.AutoArrayHashMapUnmanaged(NodeId, void){});
            return .{
                .arena = arena,
                .node_neighbors = node_neighbors,
            };
        }

        pub fn deinit(self: *Builder) void {
            const alloc = self.arena.child_allocator;
            self.arena.deinit();
            alloc.destroy(self.arena);
        }

        pub fn feed(self: *Builder, way: Way) !void {
            const arena_alloc = self.arena.allocator();
            for (way.node_ids, 0..) |node_id, i| {
                if (i > 0) {
                    try self.node_neighbors[node_id.value].put(arena_alloc, way.node_ids[i - 1], {});
                }

                if (i < way.node_ids.len - 1) {
                    try self.node_neighbors[node_id.value].put(arena_alloc, way.node_ids[i + 1], {});
                }
            }
        }

        pub fn build(self: *Builder) !NodeAdjacencyMap {
            return try NodeAdjacencyMap.init(self.arena.child_allocator, self.node_neighbors);
        }
    };

    pub fn init(alloc: Allocator, node_neighbors: []std.AutoArrayHashMapUnmanaged(NodeId, void)) !NodeAdjacencyMap {
        var storage = std.ArrayList(NodeId).init(alloc);
        defer storage.deinit();

        var segment_starts = std.ArrayList(u32).init(alloc);
        defer segment_starts.deinit();

        for (node_neighbors) |neighbors| {
            try segment_starts.append(@intCast(storage.items.len));
            try storage.appendSlice(neighbors.keys());
        }
        try segment_starts.append(@intCast(storage.items.len));

        return .{
            .storage = try storage.toOwnedSlice(),
            .segment_starts = try segment_starts.toOwnedSlice(),
        };
    }

    pub fn deinit(self: *NodeAdjacencyMap, alloc: Allocator) void {
        alloc.free(self.storage);
        alloc.free(self.segment_starts);
    }

    pub fn getNeighbors(self: *const NodeAdjacencyMap, node: NodeId) []const NodeId {
        const start = self.segment_starts[node.value];
        const end = self.segment_starts[node.value + 1];

        return self.storage[start..end];
    }

    pub fn numNodes(self: *const NodeAdjacencyMap) usize {
        return self.segment_starts.len - 1;
    }
};

pub const NodePair = struct {
    a: NodeId,
    b: NodeId,
};

pub fn NodePairMap(comptime T: type) type {
    return struct {
        inner: std.AutoHashMap(NodePair, T),

        const Self = @This();

        pub fn init(alloc: Allocator) Self {
            return .{
                .inner = std.AutoHashMap(NodePair, T).init(alloc),
            };
        }

        pub fn deinit(self: *Self) void {
            self.inner.deinit();
        }

        pub fn put(self: *Self, a: NodeId, b: NodeId, value: T) !void {
            try self.inner.put(makeNodePair(a, b), value);
        }

        pub fn get(self: *const Self, a: NodeId, b: NodeId) ?T {
            return self.inner.get(makeNodePair(a, b));
        }

        fn makeNodePair(a: NodeId, b: NodeId) NodePair {
            const larger = @max(a.value, b.value);
            const smaller = @min(a.value, b.value);
            return .{
                .a = .{ .value = smaller },
                .b = .{ .value = larger },
            };
        }
    };
}

pub const NodePairCostMultiplierMap = NodePairMap(f32);

pub const IndexBufferIt = struct {
    data: []const u32,
    i: usize,

    pub fn init(data: []const u32) IndexBufferIt {
        return .{
            .data = data,
            .i = 0,
        };
    }

    pub fn next(self: *IndexBufferIt) ?IndexRange {
        self.i += 1;
        if (self.i >= self.data.len) {
            return null;
        }

        const start = self.i;
        const slice_rel_start: []const u32 = self.data[self.i..];
        const end_rel_start = std.mem.indexOfScalar(u32, slice_rel_start, 0xffffffff);
        const end = if (end_rel_start) |v| start + v else self.data.len;
        self.i = end;
        return .{
            .start = start,
            .end = end,
        };
    }
};

pub const StringTableId = usize;

pub const StringTable = struct {
    data: []const []const u8,

    pub fn init(alloc: Allocator, buf: []const u8) !StringTable {
        var data = std.ArrayList([]const u8).init(alloc);
        defer data.deinit();

        var it: usize = 0;
        while (it < buf.len) {
            comptime std.debug.assert(builtin.cpu.arch.endian() == .little);
            const len_end = it + 2;
            if (len_end >= buf.len) {
                return error.InvalidData;
            }

            const str_len = std.mem.bytesToValue(u16, buf[it..len_end]);
            const str_end = str_len + len_end;
            defer it = str_end;

            if (str_end > buf.len) {
                return error.InvalidData;
            }

            const s = buf[len_end..str_end];
            try data.append(s);
        }

        return .{ .data = try data.toOwnedSlice() };
    }

    pub fn deinit(self: *StringTable, alloc: Allocator) void {
        alloc.free(self.data);
    }

    pub fn get(self: *const StringTable, id: StringTableId) []const u8 {
        return self.data[id];
    }

    pub fn findByPointerAddress(self: *const StringTable, p: [*]const u8) StringTableId {
        for (self.data, 0..) |item, i| {
            if (item.ptr == p) {
                return i;
            }
        }

        @panic("No id");
    }

    pub fn findByStringContent(self: *const StringTable, s: []const u8) StringTableId {
        for (self.data, 0..) |item, i| {
            if (std.mem.eql(u8, item, s)) {
                return i;
            }
        }

        @panic("No id");
    }
};

pub const MeterMetadata = struct {
    width: f32,
    height: f32,
};

pub fn latLongToMeters(point_data: []f32, metadata: Metadata) MeterMetadata {
    const converter = CoordinateSpaceConverter.init(&metadata);

    for (0..point_data.len / 2) |i| {
        const lon = &point_data[i * 2];
        const lat = &point_data[i * 2 + 1];

        lat.* = converter.latToM(lat.*);
        lon.* = converter.lonToM(lon.*);
    }

    return .{
        .width = converter.widthM(),
        .height = converter.heightM(),
    };
}

pub const CoordinateSpaceConverter = struct {
    metadata: *const Metadata,
    width_deg: f32,
    height_deg: f32,
    lat_step: f32,
    lon_step: f32,

    pub fn init(metadata: *const Metadata) CoordinateSpaceConverter {
        const center_lat = (metadata.min_lat + metadata.max_lat) / 2.0 * std.math.rad_per_deg;

        const lat_step = 111132.92 - 559.82 * @cos(2 * center_lat) + 1.175 * @cos(4 * center_lat) - 0.0023 * @cos(6 * center_lat);
        const lon_step = 111412.84 * @cos(center_lat) - 93.5 * @cos(3 * center_lat) + 0.118 * @cos(5 * center_lat);

        const width_deg = metadata.max_lon - metadata.min_lon;
        const height_deg = metadata.max_lat - metadata.min_lat;

        return .{
            .metadata = metadata,
            .lat_step = lat_step,
            .lon_step = lon_step,
            .width_deg = width_deg,
            .height_deg = height_deg,
        };
    }

    pub fn latToM(self: *const CoordinateSpaceConverter, lat: f32) f32 {
        return self.lat_step * (lat - self.metadata.min_lat);
    }

    pub fn lonToM(self: *const CoordinateSpaceConverter, lon: f32) f32 {
        return self.lon_step * (lon - self.metadata.min_lon);
    }

    pub fn widthM(self: *const CoordinateSpaceConverter) f32 {
        return self.lon_step * self.width_deg;
    }

    pub fn heightM(self: *const CoordinateSpaceConverter) f32 {
        return self.lat_step * self.height_deg;
    }
};

pub const MapDataComponents = struct {
    point_data: []f32,
    index_data: []const u32,
    string_table_data: []const u8,

    pub fn init(data: []u8, metadata: Metadata) MapDataComponents {
        return .{
            .point_data = @alignCast(std.mem.bytesAsSlice(f32, data[0..@intCast(metadata.end_nodes)])),
            .index_data = @alignCast(std.mem.bytesAsSlice(u32, data[@intCast(metadata.end_nodes)..@intCast(metadata.end_ways)])),
            .string_table_data = data[@intCast(metadata.end_ways)..],
        };
    }
};

pub const WaysForTagPair = struct {
    metadata: *const Metadata,
    ways: *const WayLookup,
    k: usize,
    v: usize,
    i: usize = 0,

    pub fn init(metadata: *const Metadata, ways: *const WayLookup, k: usize, v: usize) WaysForTagPair {
        return .{
            .metadata = metadata,
            .ways = ways,
            .k = k,
            .v = v,
        };
    }

    pub fn next(self: *WaysForTagPair) ?Way {
        while (true) {
            if (self.i >= self.metadata.way_tags.len) {
                return null;
            }
            defer self.i += 1;

            const way_tags = self.metadata.way_tags[self.i];
            if (wayTagsContains(way_tags, self.k, self.v)) {
                return self.ways.ways[self.i];
            }
        }
    }
};

pub const NodePairsForParentTagPair = struct {
    metadata: *const Metadata,
    point_pair_to_parent: *const NodePairMap(WayId),
    adjacency_map: *const NodeAdjacencyMap,
    k: usize,
    v: usize,
    node_idx: NodeId = .{ .value = 0 },
    neighbor_idx: usize = 0,

    pub fn init(k: usize, v: usize, metadata: *const Metadata, point_pair_to_parent: *const NodePairMap(WayId), adjacency_map: *const NodeAdjacencyMap) NodePairsForParentTagPair {
        return .{
            .k = k,
            .v = v,
            .metadata = metadata,
            .point_pair_to_parent = point_pair_to_parent,
            .adjacency_map = adjacency_map,
        };
    }

    pub fn next(self: *NodePairsForParentTagPair) ?NodePair {
        while (true) {
            if (self.node_idx.value >= self.adjacency_map.numNodes()) {
                return null;
            }

            const neighbors = self.adjacency_map.getNeighbors(self.node_idx);
            if (self.neighbor_idx >= neighbors.len) {
                self.node_idx.value += 1;
                self.neighbor_idx = 0;
                continue;
            }
            defer self.neighbor_idx += 1;

            const neighbor = neighbors[self.neighbor_idx];

            // We only want to see each node pair once
            if (neighbor.value > self.node_idx.value) {
                continue;
            }

            const parent = self.point_pair_to_parent.get(self.node_idx, neighbor) orelse continue;

            const tags = self.metadata.way_tags[parent.value];
            if (wayTagsContains(tags, self.k, self.v)) {
                return .{
                    .a = self.node_idx,
                    .b = neighbor,
                };
            }
        }
    }
};

pub fn wayTagsContains(tags: Metadata.Tags, k: usize, v: usize) bool {
    for (0..tags[0].len) |i| {
        const tag_k = tags[0][i];
        const tag_v = tags[1][i];

        if (tag_k == k and tag_v == v) {
            return true;
        }
    }

    return false;
}

fn wayParentScore(way: Way, point_lookup: *const PointLookup, a: Point, b: Point) f32 {
    var min_score = std.math.inf(f32);

    const ab = a.sub(b);
    const ab_len = ab.length();
    const ab_norm = ab.mul(1.0 / ab_len);

    for (0..way.node_ids.len - 1) |i| {
        const way_a = way.node_ids[i];
        const way_b = way.node_ids[i + 1];
        const way_point_a = point_lookup.get(way_a);
        const way_point_b = point_lookup.get(way_b);

        const way_ab = way_point_a.sub(way_point_b).normalized();

        const dot = @abs(ab_norm.dot(way_ab));
        const dot_thresh = 1.0 / std.math.sqrt2;
        if (dot < dot_thresh) {
            continue;
        }

        const parallel_score = (1 / dot - 1) * 5;

        // Maybe we should sum min distances to way
        const midpoint = b.add(ab.mul(0.5));
        const dist_score = lin.closestPointOnLine(midpoint, way_point_a, way_point_b).sub(midpoint).length();
        min_score = @min(dist_score + parallel_score, min_score);
    }

    return min_score;
}

pub fn findSidewalkStreets(
    alloc: Allocator,
    point_lookup: *const PointLookup,
    way_buckets: *const WayBuckets,
    string_table: *const StringTable,
    ways: *const WayLookup,
    metadata: *const Metadata,
) !PointPairToWayMap {
    const sidewalk_k = string_table.findByStringContent("highway");
    const sidewalk_v = string_table.findByStringContent("footway");

    var point_pair_to_parent = PointPairToWayMap.init(alloc);
    errdefer point_pair_to_parent.deinit();

    var way_it = WaysForTagPair.init(metadata, ways, sidewalk_k, sidewalk_v);

    while (way_it.next()) |sidewalk| {
        for (0..sidewalk.node_ids.len - 1) |i| {
            const node_id = sidewalk.node_ids[i];
            const neighbor_id = sidewalk.node_ids[i + 1];

            const point = point_lookup.get(node_id);
            const neighbor_point = point_lookup.get(neighbor_id);

            const bucket = way_buckets.get(point.y, point.x);

            var min_way_id: WayId = undefined;
            var min_way_score = std.math.inf(f32);
            for (bucket) |other_way_id| {
                const other_way_tags = metadata.way_tags[other_way_id.value];
                if (wayTagsContains(other_way_tags, sidewalk_k, sidewalk_v)) {
                    continue;
                }

                const score = wayParentScore(ways.get(other_way_id), point_lookup, point, neighbor_point);
                if (score < min_way_score) {
                    min_way_score = score;
                    min_way_id = other_way_id;
                }
            }

            if (min_way_score != std.math.inf(f32)) {
                try point_pair_to_parent.put(node_id, neighbor_id, min_way_id);
            }
        }
    }

    return point_pair_to_parent;
}

pub fn parseIndexBuffer(
    alloc: Allocator,
    point_lookup: PointLookup,
    width: f32,
    height: f32,
    index_buffer: []const u32,
) !struct { WayLookup, WayBuckets, NodeAdjacencyMap } {
    var ways = WayLookup.Builder.init(alloc, index_buffer);
    defer ways.deinit();

    var way_buckets_builder = try WayBuckets.Builder.init(alloc, width, height);
    var it = IndexBufferIt.init(index_buffer);
    var way_id: WayId = .{ .value = 0 };

    var node_neighbors = try NodeAdjacencyMap.Builder.init(alloc, point_lookup.numPoints());
    defer node_neighbors.deinit();

    while (it.next()) |idx_buf_range| {
        defer way_id.value += 1;
        const way = Way.fromIndexRange(idx_buf_range, index_buffer);

        try ways.feed(way);
        try node_neighbors.feed(way);

        for (way.node_ids) |node_id| {
            const gps_pos = point_lookup.get(node_id);
            try way_buckets_builder.push(way_id, gps_pos.y, gps_pos.x);
        }
    }

    const node_adjacency_map = try node_neighbors.build();
    const way_lookup = try ways.build();
    const way_buckets = try way_buckets_builder.build();
    return .{ way_lookup, way_buckets, node_adjacency_map };
}

pub fn MapBuckets(comptime T: type) type {
    return struct {
        const x_buckets = 150;
        const y_buckets = 150;
        const BucketId = struct {
            value: usize,
        };

        const Self = @This();

        pub const Builder = struct {
            const ItemSet = std.AutoArrayHashMapUnmanaged(T, void);

            sets: []ItemSet,
            alloc: Allocator,
            inner: Self,

            pub fn init(alloc: Allocator, width: f32, height: f32) !Builder {
                const sets = try alloc.alloc(ItemSet, x_buckets * y_buckets);
                for (sets) |*bucket| {
                    bucket.* = .{};
                }

                return .{
                    .sets = sets,
                    .alloc = alloc,
                    .inner = .{
                        .buckets = &.{},
                        .width = width,
                        .height = height,
                    },
                };
            }

            // Should only be called if build is not called or does not succeed
            pub fn deinit(self: *Builder) void {
                for (self.sets) |*set| {
                    set.deinit(self.alloc);
                }
                self.alloc.free(self.sets);
            }

            pub fn build(self: *Builder) !Self {
                const inner_buckets = try self.alloc.alloc([]T, x_buckets * y_buckets);
                var i: usize = 0;
                errdefer {
                    for (0..i) |j| {
                        self.alloc.free(inner_buckets[j]);
                    }
                    self.alloc.free(inner_buckets);
                }

                while (i < self.sets.len) {
                    defer i += 1;
                    inner_buckets[i] = try self.alloc.dupe(T, self.sets[i].keys());
                }

                defer self.deinit();
                self.inner.buckets = inner_buckets;
                return self.inner;
            }

            pub fn push(self: *Builder, item: T, lat: f32, long: f32) !void {
                const idx = latLongToBucket(lat, long, self.inner.height, self.inner.width);
                try self.sets[idx.value].put(self.alloc, item, {});
            }
        };

        buckets: [][]T,
        width: f32,
        height: f32,

        fn latLongToBucket(lat: f32, lon: f32, height: f32, width: f32) BucketId {
            const row_f = lat / height * y_buckets;
            const col_f = lon / width * x_buckets;
            var row: usize = @intFromFloat(row_f);
            var col: usize = @intFromFloat(col_f);

            if (row >= x_buckets) {
                row = x_buckets - 1;
            }

            if (col >= y_buckets) {
                col = y_buckets - 1;
            }
            return .{ .value = row * x_buckets + col };
        }

        pub fn deinit(self: *Self, alloc: Allocator) void {
            for (self.buckets) |bucket| {
                alloc.free(bucket);
            }
            alloc.free(self.buckets);
        }

        pub fn get(self: *const Self, lat: f32, long: f32) []T {
            const bucket = latLongToBucket(lat, long, self.height, self.width);
            return self.buckets[bucket.value];
        }
    };
}
