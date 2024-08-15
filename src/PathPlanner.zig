const std = @import("std");
const map_data = @import("map_data.zig");
const Allocator = std.mem.Allocator;
const PointLookup = map_data.PointLookup;
const NodeAdjacencyMap = map_data.NodeAdjacencyMap;
const NodeId = map_data.NodeId;
const ClosestStopLookup = @import("ClosestStopLookup.zig");
const lin = @import("lin.zig");
const Point = lin.Point;
const Vec = lin.Vec;
const WayId = map_data.WayId;
const WayLookup = map_data.WayLookup;
const TransitTripTimes = map_data.TransitTripTimes;

// With momentum, the a star nodes are not singular positions, but nodes
// representing where we are and where we came from. This is necessary for each
// node to have individual turning costs.
//
// LinearAstarLookup allows us to represent all combinations of nodes with
// their neighbors without the cost of expensive hashmap lookups
//
// This happens to line up with the storage format of the NodeAdjacencyMap, so
// we yoink their internal data. If they change, we'll have to calculate
// offsets ourselves
fn LinearAstarLookup(comptime T: type) type {
    return struct {
        storage: []T,
        neighbor_ids: []const NodeId,
        segment_starts: []const u32,

        const Self = @This();
        fn init(alloc: Allocator, adjacency_map: *const NodeAdjacencyMap, enable_transit_integration: bool) !Self {
            // HACK HACK HACK
            const num_nodes = adjacency_map.osm_segment_starts.len - 1;

            var segment_starts = try alloc.alloc(u32, num_nodes + 1);
            errdefer alloc.free(segment_starts);

            var neighbor_ids = try std.ArrayList(NodeId).initCapacity(alloc, adjacency_map.to_osm_storage.len);
            defer neighbor_ids.deinit();

            for (0..num_nodes) |i| {
                segment_starts[i] = @intCast(neighbor_ids.items.len);
                const node_id: NodeId = .{ .value = @intCast(i) };
                try neighbor_ids.appendSlice(adjacency_map.getOsmNeighbors(node_id));

                if (enable_transit_integration) {
                    try neighbor_ids.appendSlice(adjacency_map.getOsmTransitNeighbors(node_id));
                    const transit_neighbors = adjacency_map.getTransitNeighbors(node_id).node_ids;

                    var deuped_node_ids = std.AutoArrayHashMapUnmanaged(NodeId, void){};
                    defer deuped_node_ids.deinit(alloc);

                    for (transit_neighbors) |tn_id| {
                        try deuped_node_ids.put(alloc, tn_id, {});
                    }

                    try neighbor_ids.appendSlice(deuped_node_ids.keys());
                }
            }
            segment_starts[num_nodes] = @intCast(neighbor_ids.items.len);

            return .{
                .storage = try alloc.alloc(T, neighbor_ids.items.len),
                .neighbor_ids = try neighbor_ids.toOwnedSlice(),
                .segment_starts = segment_starts,
            };
        }

        fn deinit(self: *Self, alloc: Allocator) void {
            alloc.free(self.segment_starts);
            alloc.free(self.neighbor_ids);
            alloc.free(self.storage);
        }

        fn get(self: *Self, node: AStarNode) *T {
            return &self.storage[self.segment_starts[node.came_from.value] + node.me.value];
        }
    };
}

const TripList = struct {
    alloc: Allocator,
    trips: std.ArrayListUnmanaged(WayId),
    first_stop_id: NodeId,
    // Indexed by node id - first_stop_id, count of trips using this stop
    stop_trip_count: []u32,
    stops: std.AutoArrayHashMapUnmanaged(NodeId, void),
    way_lookup: *const map_data.WayLookup,

    fn init(alloc: Allocator, trips: []WayId, way_lookup: *const map_data.WayLookup, time_thresh: u32, transit_trip_times: TransitTripTimes, first_stop_id: NodeId, num_stops: usize) !TripList {
        var trips_al = std.ArrayListUnmanaged(WayId){};
        errdefer trips_al.deinit(alloc);

        const stop_trip_count = try alloc.alloc(u32, num_stops);
        @memset(stop_trip_count, 0);
        errdefer alloc.free(stop_trip_count);

        for (trips) |trip| {
            const times = transit_trip_times.getTransitTimes(trip);
            std.debug.assert(std.sort.isSorted(u32, times, {}, std.sort.asc(u32)));

            if (times[0] >= time_thresh) {
                try trips_al.append(alloc, trip);

                const way = way_lookup.get(trip);
                for (way.node_ids) |stop_id| {
                    stop_trip_count[stop_id.value - first_stop_id.value] += 1;
                }
            }
        }

        var ret = TripList{
            .alloc = alloc,
            .trips = trips_al,
            .first_stop_id = first_stop_id,
            .stops = std.AutoArrayHashMapUnmanaged(NodeId, void){},
            .stop_trip_count = stop_trip_count,
            .way_lookup = way_lookup,
        };

        try ret.updateStops();
        return ret;
    }

    fn deinit(self: *TripList) void {
        self.trips.deinit(self.alloc);
        self.stops.deinit(self.alloc);
        self.alloc.free(self.stop_trip_count);
    }

    fn getTripStops(self: *TripList) []NodeId {
        // potential optimization: ensure keys are sorted
        return self.stops.keys();
    }

    fn wayContainsNode(self: *const TripList, way_id: WayId, node: NodeId) bool {
        const way = self.way_lookup.get(way_id);
        for (way.node_ids) |stop| {
            if (stop.value == node.value) {
                return true;
            }
        }

        return false;
    }

    fn markVisited(self: *TripList, node: NodeId) !bool {
        if (!self.stops.contains(node)) {
            return false;
        }

        var i: usize = 0;

        while (i < self.trips.items.len) {
            const trip = self.trips.items[i];
            if (self.wayContainsNode(trip, node)) {
                _ = self.trips.swapRemove(i);
                const trip_stops = self.way_lookup.get(trip).node_ids;
                for (trip_stops) |stop_id| {
                    self.stop_trip_count[stop_id.value - self.first_stop_id.value] -= 1;
                }
                continue;
            }
            i += 1;
        }

        self.updateStops() catch @panic("rollback unimplemented so we crash sorry");
        return true;
    }

    fn updateStops(self: *TripList) !void {
        var new_stops = std.AutoArrayHashMapUnmanaged(NodeId, void){};
        errdefer new_stops.deinit(self.alloc);

        for (self.stop_trip_count, self.first_stop_id.value..) |count, node_id| {
            if (count > 0) {
                try new_stops.put(self.alloc, .{ .value = @intCast(node_id) }, {});
            }
        }

        self.stops.deinit(self.alloc);
        self.stops = new_stops;
    }
};

const GScores = LinearAstarLookup(f32);
const CameFrom = LinearAstarLookup(AStarNode);

const TransitState = union(enum) {
    none: void,
    some: struct {
        trip_list: TripList,
        trip_times: *const TransitTripTimes,
        closest_stop_lookup: ClosestStopLookup,
    },

    fn deinit(self: *TransitState) void {
        if (self.* == .none) {
            return;
        }

        self.some.trip_list.deinit();
        self.some.closest_stop_lookup.deinit();
    }

    fn markVisited(self: *TransitState, node_id: NodeId) !void {
        switch (self.*) {
            .none => {},
            .some => |*state| {
                if (try state.trip_list.markVisited(node_id)) {
                    try state.closest_stop_lookup.updateStops(state.trip_list.getTripStops());
                }
            },
        }
    }
};

alloc: Allocator,
points: *const PointLookup,
adjacency_map: *const NodeAdjacencyMap,
node_costs: *const map_data.NodePairCostMultiplierMap,
gscores: GScores,
came_from: CameFrom,
turning_cost: f32,
min_cost_multiplier: f32,
path_start_time: u32,
movement_speed: f32,
transit_state: TransitState,
q: std.PriorityQueue(NodeWithFscore, void, order),
start: NodeId,
end: NodeId,
final_node: ?NodeWithFscore = null,

const PathPlanner = @This();

const NeighborIndex = struct {
    value: u8,
};

const AStarNode = struct {
    me: NeighborIndex,
    came_from: NodeId,
};

const NodeWithFscore = struct {
    id: AStarNode,
    fscore: f32,
    time_elapsed: f32,
};

pub fn init(
    alloc: Allocator,
    points: *const PointLookup,
    way_lookup: *const WayLookup,
    adjacency_map: *const NodeAdjacencyMap,
    transit_trip_times: *const TransitTripTimes,
    node_costs: *const map_data.NodePairCostMultiplierMap,
    meter_metadata: map_data.MeterMetadata,
    start: NodeId,
    end: NodeId,
    turning_cost: f32,
    min_cost_multiplier: f32,
    path_start_time: u32,
    movement_speed: f32,
    enable_transit_integration: bool,
) !PathPlanner {
    var gscores = try GScores.init(alloc, adjacency_map, enable_transit_integration);
    @memset(gscores.storage, std.math.inf(f32));
    errdefer gscores.deinit(alloc);

    var came_from = try CameFrom.init(alloc, adjacency_map, enable_transit_integration);
    errdefer came_from.deinit(alloc);

    var q = std.PriorityQueue(NodeWithFscore, void, PathPlanner.order).init(alloc, {});
    errdefer q.deinit();
    try q.ensureTotalCapacity(500);

    var trips = try alloc.alloc(WayId, transit_trip_times.end_transit_way_id.value - transit_trip_times.first_transit_way_id.value);
    defer alloc.free(trips);

    for (0..trips.len) |i| {
        trips[i] = .{ .value = i + transit_trip_times.first_transit_way_id.value };
    }

    var transit_state: TransitState = .none;

    if (enable_transit_integration) {
        var trip_list = try TripList.init(alloc, trips, way_lookup, path_start_time, transit_trip_times.*, points.first_transit_id, points.numTransitPoints());
        errdefer trip_list.deinit();

        const stops = trip_list.getTripStops();
        const closest_stop_lookup = try ClosestStopLookup.init(alloc, stops, points, meter_metadata.width, meter_metadata.height);
        errdefer closest_stop_lookup.deinit();

        transit_state = .{
            .some = .{
                .trip_list = trip_list,
                .trip_times = transit_trip_times,
                .closest_stop_lookup = closest_stop_lookup,
            },
        };
    }

    var ret = PathPlanner{
        .alloc = alloc,
        .points = points,
        .adjacency_map = adjacency_map,
        .node_costs = node_costs,
        .gscores = gscores,
        .came_from = came_from,
        .transit_state = transit_state,
        .q = q,
        .start = start,
        .end = end,
        .turning_cost = turning_cost,
        .path_start_time = path_start_time,
        .movement_speed = movement_speed,
        .min_cost_multiplier = min_cost_multiplier,
    };

    try ret.q.add(.{
        .id = .{
            .me = .{ .value = 0xff },
            .came_from = start,
        },
        .fscore = ret.distance(start, end),
        .time_elapsed = 0.0,
    });

    return ret;
}

pub fn deinit(self: *PathPlanner) void {
    self.gscores.deinit(self.alloc);
    self.came_from.deinit(self.alloc);
    self.q.deinit();
    self.transit_state.deinit();
}

fn distance(self: *PathPlanner, a_id: NodeId, b_id: NodeId) f32 {
    const a = self.points.get(a_id);
    const b = self.points.get(b_id);

    return b.sub(a).length();
}

fn heuristicDistance(self: *PathPlanner, id: NodeId) f32 {
    if (self.transit_state == .none) {
        return self.distance(id, self.end);
    }

    const pos = self.points.get(id);

    return @min(
        self.distance(id, self.end),
        self.transit_state.some.closest_stop_lookup.getDistance(pos),
    );
}

fn order(_: void, a: NodeWithFscore, b: NodeWithFscore) std.math.Order {
    return std.math.order(a.fscore, b.fscore);
}

fn reconstructPath(self: *PathPlanner, end: AStarNode) ![]const NodeId {
    var ret = std.ArrayList(NodeId).init(self.alloc);
    defer ret.deinit();

    try ret.append(self.end);

    var it = end;
    while (it.came_from.value != self.start.value) {
        try ret.append(it.came_from);
        it = self.came_from.get(it).*;
    }

    try ret.append(self.start);
    return try ret.toOwnedSlice();
}

fn turningCost(self: *const PathPlanner, ab: Vec, bc: Vec) f32 {
    const ab_len = ab.length();
    const bc_len = bc.length();
    if (ab_len == 0 or bc_len == 0) {
        return 0.0;
    }

    const ab_norm = ab.mul(1.0 / ab_len);
    const bc_norm = bc.mul(1.0 / bc_len);

    const turning_amount = 0.5 - ab_norm.dot(bc_norm) / 2.0;

    return turning_amount * self.turning_cost;
}

fn updateOsmNeighbor(self: *PathPlanner, current_id: NodeId, neighbor_id: NodeId, current: AStarNode, prev_time_elapsed: f32, current_score: f32) !void {
    const current_point = self.points.get(current_id);
    const current_from_point = self.points.get(current.came_from);

    const neighbor_point = self.points.get(neighbor_id);

    const this_turning_cost = self.turningCost(
        current_point.sub(current_from_point),
        neighbor_point.sub(current_point),
    );

    const node_cost = self.node_costs.get(current_id, neighbor_id) orelse 1.0;
    const neighbor_dist = self.distance(current_id, neighbor_id);
    const tentative_score = current_score + neighbor_dist / self.movement_speed * node_cost + this_turning_cost;

    try self.updateNeighborWithScore(tentative_score, current, current_id, neighbor_id, prev_time_elapsed + neighbor_dist / self.movement_speed);
}

fn updateTransitNeighbor(self: *PathPlanner, current_id: NodeId, neighbor_id: NodeId, current: AStarNode, departure_time: u32, prev_time_elapsed: f32, current_score: f32) !void {
    try self.transit_state.markVisited(neighbor_id);

    if (departure_time <= self.path_start_time + @as(u32, @intFromFloat(@ceil(prev_time_elapsed)))) {
        return;
    }

    const departure_time_f: f32 = @floatFromInt(departure_time);
    const path_start_time_f: f32 = @floatFromInt(self.path_start_time);
    const cost: f32 = departure_time_f - path_start_time_f - prev_time_elapsed;
    const cost_multiplier = self.node_costs.get(current_id, neighbor_id) orelse 1.0;
    const tentative_score = current_score + cost * cost_multiplier;

    try self.updateNeighborWithScore(
        tentative_score,
        current,
        current_id,
        neighbor_id,
        departure_time_f - path_start_time_f,
    );
}

fn updateNeighborWithScore(self: *PathPlanner, tentative_score: f32, current: AStarNode, current_id: NodeId, neighbor_id: NodeId, new_time_elapsed: f32) !void {
    const neighbor_idx = self.findMyNeighborIndex(current_id, neighbor_id);
    const neighbor_a_star_id = AStarNode{
        .me = neighbor_idx,
        .came_from = current_id,
    };
    const neighbor_entry = self.gscores.get(neighbor_a_star_id);
    if (tentative_score >= neighbor_entry.*) {
        return;
    }

    neighbor_entry.* = tentative_score;
    const fscore = tentative_score + self.heuristicDistance(neighbor_id) / self.movement_speed * self.min_cost_multiplier;
    self.came_from.get(neighbor_a_star_id).* = current;

    const neighbor_w_fscore = NodeWithFscore{
        .id = neighbor_a_star_id,
        .fscore = fscore,
        .time_elapsed = new_time_elapsed,
    };

    try self.q.add(neighbor_w_fscore);
}

fn findMyNeighborIndex(self: *const PathPlanner, me: NodeId, neighbor: NodeId) NeighborIndex {
    const start = self.gscores.segment_starts[me.value];
    const end = self.gscores.segment_starts[me.value + 1];
    const neighbor_neighbors = self.gscores.neighbor_ids[start..end];
    for (neighbor_neighbors, 0..) |node_id, i| {
        if (node_id.value == neighbor.value) {
            return .{ .value = @intCast(i) };
        }
    }

    @panic("No neighbor");
}

pub const PlannedPath = struct {
    path: []const NodeId,
    time: f32,

    pub fn deinit(self: *const PlannedPath, alloc: Allocator) void {
        alloc.free(self.path);
    }
};

pub fn step(self: *PathPlanner) !?PlannedPath {
    if (self.final_node) |n| {
        const path = try self.reconstructPath(n.id);
        return .{
            .path = path,
            .time = n.time_elapsed,
        };
    }

    const current_node_id = self.q.removeOrNull() orelse return error.NoPath;
    const idx = self.gscores.segment_starts[current_node_id.id.came_from.value] + current_node_id.id.me.value;
    const me_node_id = if (current_node_id.id.came_from.value == self.start.value and current_node_id.id.me.value == 0xff) self.start else self.gscores.neighbor_ids[idx];

    if (me_node_id.value == self.end.value) {
        self.final_node = current_node_id;
        const path = try self.reconstructPath(current_node_id.id);
        return .{
            .path = path,
            .time = current_node_id.time_elapsed,
        };
    }

    const neighbors = self.adjacency_map.getOsmNeighbors(me_node_id);

    var current_gscore = self.gscores.get(current_node_id.id).*;
    if (current_node_id.id.came_from.value == self.start.value and current_node_id.id.me.value == 0xff) {
        current_gscore = 0.0;
    }

    for (neighbors) |neighbor| {
        try self.updateOsmNeighbor(me_node_id, neighbor, current_node_id.id, current_node_id.time_elapsed, current_gscore);
    }

    if (self.transit_state == .some) {
        const osm_to_transit_neighbors = self.adjacency_map.getOsmTransitNeighbors(me_node_id);
        for (osm_to_transit_neighbors) |neighbor| {
            try self.transit_state.markVisited(neighbor);
            try self.updateOsmNeighbor(me_node_id, neighbor, current_node_id.id, current_node_id.time_elapsed, current_gscore);
        }

        const transit_neighbors = self.adjacency_map.getTransitNeighbors(me_node_id);
        for (0..transit_neighbors.node_ids.len) |i| {
            const neighbor = transit_neighbors.node_ids[i];
            const departure_time = transit_neighbors.departure_times[i];
            try self.updateTransitNeighbor(me_node_id, neighbor, current_node_id.id, departure_time, current_node_id.time_elapsed, current_gscore);
        }
    }

    return null;
}

pub fn run(self: *PathPlanner) !PlannedPath {
    while (true) {
        if (try self.step()) |val| {
            return val;
        }
    }
}
