const std = @import("std");
const Allocator = std.mem.Allocator;
const Metadata = @import("Metadata.zig");
const MouseTracker = @import("MouseTracker.zig");
const map_data = @import("map_data.zig");
const lin = @import("lin.zig");
const PathPlanner = @import("PathPlanner.zig");
const Renderer = @import("Renderer.zig");
const TextureRenderer = @import("TextureRenderer.zig");
const gl_utils = @import("gl_utils.zig");
const monitored_attributes = @import("monitored_attributes.zig");
const HeuristicRenderer = @import("HeuristicRenderer.zig");
const Gl = gl_utils.Gl;
const image_tile_data = @import("image_tile_data.zig");
const ImageTileData = image_tile_data.ImageTileData;
const Point = lin.Point;
const Vec = lin.Vec;
const MapPos = lin.Point;
const PointLookup = map_data.PointLookup;
const NodeAdjacencyMap = map_data.NodeAdjacencyMap;
const NodeId = map_data.NodeId;
const WayLookup = map_data.WayLookup;
const Way = map_data.Way;
const WayId = map_data.WayId;
const StringTable = map_data.StringTable;
const gui = @import("gui_bindings.zig");
const ViewState = gl_utils.ViewState;
const WaysForTagPair = map_data.WaysForTagPair;
const WayBuckets = map_data.WayBuckets;

const App = @This();

const WayFindingDebugElem = struct {
    size: f32,
    pos: Point,
};

const RenderSnapshot = struct {
    view_state: ViewState = undefined,

    debug_points: []NodeId = &.{},

    closest_node: NodeId = .{ .value = 0 },
    closest_way: ?WayId = null,
    closest_way_nodes: ?[2]NodeId = null,

    neighbor_points: []const NodeId = &.{},

    parent_way: ?WayId = null,

    // Probably can turn into vbo or ebo
    way_finding_debug: std.ArrayListUnmanaged(WayFindingDebugElem) = .{},
    closest_way_point: ?Point = null,

    path_start: ?NodeId = null,
    path_end: ?NodeId = null,

    planned_path: []const NodeId = &.{},

    render_heuristic: bool = false,

    fn deinit(self: *RenderSnapshot, alloc: Allocator) void {
        alloc.free(self.debug_points);
        alloc.free(self.planned_path);
        self.way_finding_debug.deinit(alloc);
    }

    fn clearPpDebugPoints(self: *RenderSnapshot, alloc: Allocator) void {
        alloc.free(self.debug_points);
        self.debug_points = &.{};
    }

    fn updatePathPlanner(self: *RenderSnapshot, alloc: Allocator, debug_enable: bool, pp: *const PathPlanner) !void {
        if (pp.final_node) |final_node| {
            const new_path = pp.reconstructPath(final_node.id) catch return;
            alloc.free(self.planned_path);
            self.planned_path = new_path;
        } else {
            alloc.free(self.planned_path);
            self.planned_path = &.{};
        }

        if (!debug_enable) {
            self.clearPpDebugPoints(alloc);
            return;
        }

        var seen_gscores = std.ArrayList(NodeId).init(alloc);
        defer seen_gscores.deinit();

        for (0..pp.gscores.segment_starts.len - 1) |i| {
            const start = pp.gscores.segment_starts[i];
            const end = pp.gscores.segment_starts[i + 1];
            for (pp.gscores.storage[start..end]) |score| {
                if (score != std.math.inf(f32)) {
                    seen_gscores.append(.{ .value = @intCast(i) }) catch return;
                    break;
                }
            }
        }

        alloc.free(self.debug_points);
        self.debug_points = try seen_gscores.toOwnedSlice();
    }
};

const AppRenderer = struct {
    renderer: Renderer,
    heuristic_renderer: HeuristicRenderer,
    texture_renderer: TextureRenderer,

    metadata: *const Metadata = undefined,
    monitored_attributes: *const monitored_attributes.MonitoredAttributeTracker = undefined,

    // Things for sure we want to keep
    points: *const PointLookup = undefined,
    ways: *const WayLookup = undefined,
    image_tile_metadata: ImageTileData,
    textures: []i32,

    pp_debug_point_buffer: i32,
    num_pp_debug_points: usize = 0,

    planned_path: i32,
    planned_path_len: usize = 0,

    snapshot: RenderSnapshot = .{},

    fn render(self: *AppRenderer) void {
        // FIXME: This should be set somewhere else
        Renderer.updatePointBuffer(self.pp_debug_point_buffer, self.snapshot.debug_points);
        self.num_pp_debug_points = self.snapshot.debug_points.len;

        Renderer.updatePointBuffer(self.planned_path, self.snapshot.planned_path);
        self.planned_path_len = self.snapshot.planned_path.len;

        gui.glClearColor(0.0, 0.0, 0.0, 1.0);
        gui.glClear(Gl.COLOR_BUFFER_BIT);

        for (self.textures, 0..) |tex, i| {
            var bound_texture_renderer = self.texture_renderer.bind();
            const screen_space = tileLocScreenSpace(self.image_tile_metadata[i], self.snapshot.view_state, self.metadata.*);
            bound_texture_renderer.render(
                tex,
                screen_space.center,
                screen_space.scale,
            );
        }

        var bound_renderer = self.renderer.bind();
        bound_renderer.render(self.snapshot.view_state);

        for (self.monitored_attributes.rendering.attributes.items) |monitored| {
            bound_renderer.inner.r.set(monitored.color.r);
            bound_renderer.inner.g.set(monitored.color.g);
            bound_renderer.inner.b.set(monitored.color.b);
            bound_renderer.renderIndexBuffer(monitored.index_buffer, monitored.index_buffer_len, Gl.LINE_STRIP);
        }

        if (self.snapshot.render_heuristic) {
            var bound = self.heuristic_renderer.bind();
            bound.render(self.snapshot.view_state);
            bound_renderer = self.renderer.bind();
        }

        const transit_point_size = 10000.0;
        bound_renderer.inner.point_size.set(transit_point_size * self.snapshot.view_state.zoom);
        const len = self.points.numPoints() - self.metadata.transit_node_start_idx;
        bound_renderer.inner.r.set(0.3);
        bound_renderer.inner.g.set(0.3);
        bound_renderer.inner.b.set(1.0);
        gui.glBindVertexArray(bound_renderer.inner.vao);
        gui.glDrawArrays(Gl.POINTS, @intCast(self.metadata.transit_node_start_idx), @intCast(len));

        bound_renderer.inner.r.set(1.0);
        bound_renderer.inner.g.set(0.0);
        bound_renderer.inner.b.set(0.0);
        bound_renderer.inner.point_size.set(10.0);
        if (self.snapshot.path_start) |s| {
            bound_renderer.renderPoints(&.{s}, Gl.POINTS);
        }

        if (self.snapshot.path_end) |end| {
            bound_renderer.renderPoints(&.{end}, Gl.POINTS);
        }

        if (self.num_pp_debug_points > 0) {
            bound_renderer.inner.point_size.set(10.0);
            bound_renderer.inner.r.set(1.0);
            bound_renderer.inner.g.set(0.0);
            bound_renderer.inner.b.set(0.0);
            bound_renderer.renderIndexBuffer(
                self.pp_debug_point_buffer,
                self.num_pp_debug_points,
                Gl.POINTS,
            );
        }

        if (self.planned_path_len > 0) {
            bound_renderer.inner.r.set(1.0);
            bound_renderer.inner.g.set(0.0);
            bound_renderer.inner.b.set(0.0);
            bound_renderer.renderIndexBuffer(self.planned_path, self.planned_path_len, Gl.LINE_STRIP);
        }

        for (self.snapshot.way_finding_debug.items) |item| {
            bound_renderer.inner.r.set(0.0);
            bound_renderer.inner.g.set(1.0);
            bound_renderer.inner.b.set(1.0);
            bound_renderer.inner.point_size.set(item.size);
            bound_renderer.renderCoords(&.{ item.pos.x, item.pos.y }, Gl.POINTS);
        }

        if (self.snapshot.parent_way) |parent| {
            bound_renderer.inner.r.set(0);
            bound_renderer.inner.g.set(0);
            bound_renderer.inner.b.set(1);
            bound_renderer.renderSelectedWay(self.ways.get(parent));
        }

        bound_renderer.inner.r.set(0.0);
        bound_renderer.inner.g.set(1.0);
        bound_renderer.inner.b.set(1.0);

        bound_renderer.inner.point_size.set(10.0);
        bound_renderer.renderPoints(self.snapshot.neighbor_points, Gl.POINTS);

        if (self.snapshot.closest_way) |way_id| {
            bound_renderer.renderSelectedWay(self.ways.get(way_id));
        }

        if (self.snapshot.closest_way_point) |closest_point| {
            bound_renderer.renderCoords(&.{ self.snapshot.view_state.center.x, self.snapshot.view_state.center.y, closest_point.x, closest_point.y }, Gl.LINE_STRIP);
        }

        bound_renderer.inner.point_size.set(10.0);
        bound_renderer.renderPoints(&.{self.snapshot.closest_node}, Gl.POINTS);
    }

    fn deinit(self: *AppRenderer, alloc: Allocator) void {
        self.snapshot.deinit(alloc);
        alloc.free(self.textures);
    }
};

alloc: Allocator,
mouse_tracker: MouseTracker = .{},
metadata: *const Metadata,
meter_metadata: map_data.MeterMetadata,
view_state: ViewState,
points: PointLookup,
ways: WayLookup,
string_table: StringTable,
adjacency_map: NodeAdjacencyMap,
way_buckets: WayBuckets,
path_planner: ?PathPlanner = null,
closest_node: NodeId = NodeId{ .value = 0 },
transit_trip_times: map_data.TransitTripTimes,
path_start: ?NodeId = null,
path_end: ?NodeId = null,
render_state: AppRenderer,
monitored_attributes: monitored_attributes.MonitoredAttributeTracker,
point_pair_to_parent: map_data.PointPairToWayMap,
turning_cost: f32 = 0.0,
path_start_time: u32 = 0,
movement_speed: f32 = 1.34112,
debug_way_finding: bool = false,
debug_point_neighbors: bool = false,
debug_path_finding: bool = false,
debug_parenting: bool = false,
enable_transit_integration: bool = false,

pub fn init(alloc: Allocator, aspect_val: f32, map_data_buf: []u8, metadata: *const Metadata, image_tile_metadata: ImageTileData) !*App {
    const split_data = map_data.MapDataComponents.init(map_data_buf, metadata.*);
    const meter_metdata = map_data.latLongToMeters(split_data.point_data, metadata.*);

    const textures = try alloc.alloc(i32, image_tile_metadata.len);
    errdefer alloc.free(textures);
    @memset(textures, -1);

    for (0..image_tile_metadata.len) |i| {
        gui.fetchTexture(i, image_tile_metadata[i].path.ptr, image_tile_metadata[i].path.len);
    }

    var string_table = try StringTable.init(alloc, split_data.string_table_data);
    errdefer string_table.deinit(alloc);

    const view_state = ViewState{
        .center = .{
            .x = meter_metdata.width / 2.0,
            .y = meter_metdata.height / 2.0,
        },
        .zoom = 2.0 / meter_metdata.width,
        .aspect = aspect_val,
    };

    const point_lookup = PointLookup{ .points = split_data.point_data, .first_transit_id = .{ .value = metadata.transit_node_start_idx } };

    const index_buffer_objs = try map_data.parseIndexBuffer(alloc, point_lookup, meter_metdata.width, meter_metdata.height, metadata, split_data.index_data);
    var way_lookup = index_buffer_objs[0];
    errdefer way_lookup.deinit(alloc);

    var way_buckets = index_buffer_objs[1];
    errdefer way_buckets.deinit(alloc);

    var adjacency_map = index_buffer_objs[2];
    errdefer adjacency_map.deinit(alloc);

    var point_pair_to_parent = try map_data.findSidewalkStreets(
        alloc,
        &point_lookup,
        &way_buckets,
        &string_table,
        &way_lookup,
        metadata,
    );
    errdefer point_pair_to_parent.deinit();

    const transit_trip_times = map_data.TransitTripTimes.init(metadata);

    var renderer = Renderer.init(
        split_data.point_data,
        split_data.index_data,
        @intCast(way_lookup.indexBufferOffset(.{ .value = metadata.transit_way_start_idx }, split_data.index_data)),
        @intCast(way_lookup.indexBufferOffset(.{ .value = metadata.osm_to_transit_way_start_idx }, split_data.index_data)),
    );
    renderer.bind().render(view_state);

    const texture_renderer = TextureRenderer.init();
    const heuristic_renderer = HeuristicRenderer.init(meter_metdata);

    const ret = try alloc.create(App);
    errdefer alloc.destroy(ret);

    ret.* = .{
        .alloc = alloc,
        .adjacency_map = adjacency_map,
        .metadata = metadata,
        .meter_metadata = meter_metdata,
        .view_state = view_state,
        .points = point_lookup,
        .ways = way_lookup,
        .way_buckets = way_buckets,
        .point_pair_to_parent = point_pair_to_parent,
        .transit_trip_times = transit_trip_times,
        .string_table = string_table,
        .render_state = .{
            .renderer = renderer,
            .texture_renderer = texture_renderer,
            .heuristic_renderer = heuristic_renderer,
            .image_tile_metadata = image_tile_metadata,
            .pp_debug_point_buffer = gui.glCreateBuffer(),
            .planned_path = gui.glCreateBuffer(),
            .textures = textures,
        },
        // Sibling reference
        .monitored_attributes = undefined,
    };

    ret.monitored_attributes = monitored_attributes.MonitoredAttributeTracker.init(alloc, metadata, &ret.ways, &ret.point_pair_to_parent, &ret.adjacency_map);

    return ret;
}

pub fn deinit(self: *App) void {
    self.adjacency_map.deinit(self.alloc);
    self.ways.deinit(self.alloc);
    self.way_buckets.deinit(self.alloc);
    self.string_table.deinit(self.alloc);
    self.monitored_attributes.deinit();
    self.point_pair_to_parent.deinit();
    self.render_state.deinit(self.alloc);
    if (self.path_planner) |*pp| {
        pp.deinit();
    }
    self.alloc.destroy(self);
}

pub fn onMouseDown(self: *App, x: f32, y: f32) void {
    self.mouse_tracker.onDown(x, y);
}

pub fn onMouseUp(self: *App) void {
    self.mouse_tracker.onUp();
}

pub fn onMouseMove(self: *App, x: f32, y: f32) !void {
    if (self.mouse_tracker.down) {
        const movement = self.mouse_tracker.getMovement(x, y).mul(2.0 / self.view_state.zoom);
        const new_pos = self.view_state.center.add(movement);

        // floating point imprecision may result in no actual movement of the
        // screen center. We should _not_ recenter in this case as it results
        // in slow mouse movements never moving the screen
        if (new_pos.y != self.view_state.center.y) {
            self.mouse_tracker.pos.y = y;
        }

        if (new_pos.x != self.view_state.center.x) {
            self.mouse_tracker.pos.x = x;
        }

        self.view_state.center = new_pos;
    }

    const potential_ways = self.way_buckets.get(
        self.view_state.center.y,
        self.view_state.center.x,
    );

    var calc = ClosestWayCalculator.init(
        self.view_state.center,
        self.ways,
        potential_ways,
        self.points,
    );

    self.render_state.snapshot.way_finding_debug.clearRetainingCapacity();
    if (self.debug_way_finding) {
        while (calc.step()) |debug| {
            const size = std.math.pow(f32, std.math.e, -debug.dist * 0.05) * 50.0;
            if (size > 1) {
                try self.render_state.snapshot.way_finding_debug.append(self.alloc, .{ .size = size, .pos = debug.dist_loc });
            }
        }
    } else {
        while (calc.step()) |_| {}
    }

    gui.clearTags();
    if (calc.min_way.value < self.metadata.way_tags.len) {
        const way_tags = self.metadata.way_tags[calc.min_way.value];
        for (0..way_tags[0].len) |i| {
            const key = self.string_table.get(way_tags[0][i]);
            const val = self.string_table.get(way_tags[1][i]);
            gui.pushTag(key.ptr, key.len, val.ptr, val.len);
        }
    }

    if (calc.min_dist != std.math.inf(f32)) {
        const way = self.ways.get(calc.min_way);
        if (calc.min_way_segment > way.node_ids.len) {
            std.log.err("invalid segment", .{});
            unreachable;
        }

        const node_id = way.node_ids[calc.min_way_segment];
        const neighbor_id = way.node_ids[calc.neighbor_segment];
        self.render_state.snapshot.closest_way_nodes = .{
            node_id,
            neighbor_id,
        };

        if (self.debug_parenting) {
            if (self.point_pair_to_parent.get(node_id, neighbor_id)) |parent| {
                self.render_state.snapshot.parent_way = parent;
            }
        }

        if (self.debug_way_finding) {
            self.render_state.snapshot.closest_way_point = calc.min_dist_loc;
        } else {
            self.render_state.snapshot.closest_way_point = null;
        }

        self.render_state.snapshot.closest_way = calc.min_way;

        self.closest_node = node_id;
        gui.setNodeId(node_id.value);
    } else {
        self.render_state.snapshot.closest_way_point = null;
        self.render_state.snapshot.closest_way = null;
    }
}

pub fn setAspect(self: *App, aspect: f32) void {
    self.view_state.aspect = aspect;
}

pub fn zoomIn(self: *App) void {
    self.view_state.zoom *= 2.0;
}

pub fn zoomOut(self: *App) void {
    self.view_state.zoom *= 0.5;
}

pub fn updateRenderState(self: *App) void {
    const pp_debug_points_available = self.render_state.num_pp_debug_points != 0;
    if (self.debug_path_finding != pp_debug_points_available) {
        if (self.path_planner) |*pp| {
            // FIXME: bad panic
            self.render_state.snapshot.updatePathPlanner(self.alloc, self.debug_path_finding, pp) catch @panic("uh oh");
        }
    }

    self.render_state.snapshot.render_heuristic = self.path_planner != null and self.debug_path_finding and self.path_planner.?.transit_state == .some;
    if (self.render_state.snapshot.render_heuristic) {
        // FIXME: bad panic
        self.render_state.heuristic_renderer.feed(self.alloc, self.path_planner.?.transit_state.some.closest_stop_lookup) catch @panic("uh oh");
    }

    if (self.debug_point_neighbors) {
        self.render_state.snapshot.neighbor_points = self.adjacency_map.getOsmNeighbors(self.closest_node);
    } else {
        self.render_state.snapshot.neighbor_points = &.{};
    }

    self.render_state.snapshot.view_state = self.view_state;
    self.render_state.metadata = self.metadata;
    // FIXME: Some of these should be long lived
    self.render_state.monitored_attributes = &self.monitored_attributes;
    self.render_state.points = &self.points;
    self.render_state.ways = &self.ways;
    self.render_state.snapshot.path_start = self.path_start;
    self.render_state.snapshot.path_end = self.path_end;
    self.render_state.snapshot.closest_node = self.closest_node;
}

pub fn render(self: *App) void {
    self.updateRenderState();
    self.render_state.render();

    //gui.glClearColor(0.0, 0.0, 0.0, 1.0);
    //gui.glClear(Gl.COLOR_BUFFER_BIT);

    //for (self.render_state.textures, 0..) |tex, i| {
    //    var bound_texture_renderer = self.render_state.texture_renderer.bind();
    //    const screen_space = tileLocScreenSpace(self.render_state.image_tile_metadata[i], self.view_state, self.metadata.*);
    //    bound_texture_renderer.render(
    //        tex,
    //        screen_space.center,
    //        screen_space.scale,
    //    );
    //}

    //var bound_renderer = self.render_state.renderer.bind();
    //bound_renderer.render(self.view_state);

    //for (self.monitored_attributes.rendering.attributes.items) |monitored| {
    //    bound_renderer.inner.r.set(monitored.color.r);
    //    bound_renderer.inner.g.set(monitored.color.g);
    //    bound_renderer.inner.b.set(monitored.color.b);
    //    bound_renderer.renderIndexBuffer(monitored.index_buffer, monitored.index_buffer_len, Gl.LINE_STRIP);
    //}

    //if (self.path_planner) |*pp| {
    //    if (self.debug_path_finding and pp.transit_state == .some) {
    //        // FIXME: Bad panic
    //        self.render_state.heuristic_renderer.feed(self.alloc, pp.transit_state.some.closest_stop_lookup) catch @panic("uh oh");

    //        var bound = self.render_state.heuristic_renderer.bind();
    //        bound.render(self.view_state);

    //        bound_renderer = self.render_state.renderer.bind();
    //    }
    //}

    //const transit_point_size = 10000.0;
    //bound_renderer.inner.point_size.set(transit_point_size * self.view_state.zoom);
    //const len = self.points.numPoints() - self.metadata.transit_node_start_idx;
    //bound_renderer.inner.r.set(0.3);
    //bound_renderer.inner.g.set(0.3);
    //bound_renderer.inner.b.set(1.0);
    //gui.glBindVertexArray(bound_renderer.inner.vao);
    //gui.glDrawArrays(Gl.POINTS, @intCast(self.metadata.transit_node_start_idx), @intCast(len));

    //bound_renderer.inner.r.set(1.0);
    //bound_renderer.inner.g.set(0.0);
    //bound_renderer.inner.b.set(0.0);
    //bound_renderer.inner.point_size.set(10.0);
    //if (self.path_start) |s| {
    //    bound_renderer.renderPoints(&.{s}, Gl.POINTS);
    //}

    //if (self.path_end) |end| {
    //    bound_renderer.renderPoints(&.{end}, Gl.POINTS);
    //}

    //if (self.path_planner) |pp| {
    //    if (self.render_state.num_pp_debug_points > 0) {
    //        bound_renderer.inner.point_size.set(10.0);
    //        bound_renderer.inner.r.set(1.0);
    //        bound_renderer.inner.g.set(0.0);
    //        bound_renderer.inner.b.set(0.0);
    //        bound_renderer.renderIndexBuffer(
    //            self.render_state.pp_debug_point_buffer,
    //            self.render_state.num_pp_debug_points,
    //            Gl.POINTS,
    //        );
    //    }

    //    if (pp.final_node) |final_node| {
    //        const res = pp.reconstructPath(final_node.id) catch return;
    //        defer self.alloc.free(res);

    //        bound_renderer.inner.r.set(1.0);
    //        bound_renderer.inner.g.set(0.0);
    //        bound_renderer.inner.b.set(0.0);
    //        bound_renderer.renderPoints(res, Gl.LINE_STRIP);
    //    }
    //}

    //for (self.render_state.way_finding_debug.items) |item| {
    //    bound_renderer.inner.r.set(0.0);
    //    bound_renderer.inner.g.set(1.0);
    //    bound_renderer.inner.b.set(1.0);
    //    bound_renderer.inner.point_size.set(item.size);
    //    bound_renderer.renderCoords(&.{ item.pos.x, item.pos.y }, Gl.POINTS);
    //}

    //if (self.debug_parenting) blk: {
    //    const nodes = self.render_state.closest_way_nodes orelse break :blk;
    //    if (self.point_pair_to_parent.get(nodes[0], nodes[1])) |parent| {
    //        bound_renderer.inner.r.set(0);
    //        bound_renderer.inner.g.set(0);
    //        bound_renderer.inner.b.set(1);
    //        bound_renderer.renderSelectedWay(self.ways.get(parent));
    //    }
    //}

    //bound_renderer.inner.r.set(0.0);
    //bound_renderer.inner.g.set(1.0);
    //bound_renderer.inner.b.set(1.0);

    //if (self.debug_point_neighbors) {
    //    const neighbors = self.adjacency_map.getOsmNeighbors(self.closest_node);
    //    bound_renderer.inner.point_size.set(10.0);
    //    bound_renderer.renderPoints(neighbors, Gl.POINTS);
    //}

    //if (self.render_state.closest_way) |way_id| {
    //    bound_renderer.renderSelectedWay(self.ways.get(way_id));
    //}

    //if (self.debug_way_finding) blk: {
    //    const closest_point = self.render_state.closest_way_point orelse break :blk;
    //    bound_renderer.renderCoords(&.{ self.view_state.center.x, self.view_state.center.y, closest_point.x, closest_point.y }, Gl.LINE_STRIP);
    //}

    //bound_renderer.inner.point_size.set(10.0);
    //bound_renderer.renderPoints(&.{self.closest_node}, Gl.POINTS);
}

pub fn startPath(self: *App) !void {
    self.path_start = self.closest_node;
    if (self.path_end) |end| {
        try self.resetPathPlanner(self.closest_node, end);
    }
}

pub fn stepPath(self: *App, amount: u32) !void {
    if (self.path_planner) |*pp| {
        for (0..amount) |_| {
            if (try pp.step()) |path| {
                path.deinit(self.alloc);
                break;
            }
        }

        try self.render_state.snapshot.updatePathPlanner(self.alloc, self.debug_path_finding, pp);
    }
}

pub fn endPath(self: *App) !void {
    self.path_end = self.closest_node;
    if (self.path_start) |s| {
        try self.resetPathPlanner(s, self.closest_node);
    }
}

fn resetPathPlanner(self: *App, start: NodeId, end: NodeId) !void {
    const new_pp = try PathPlanner.init(
        self.alloc,
        &self.points,
        &self.ways,
        &self.adjacency_map,
        &self.transit_trip_times,
        &self.monitored_attributes.cost.node_costs,
        self.meter_metadata,
        start,
        end,
        self.turning_cost,
        self.monitored_attributes.cost.min_cost_multiplier,
        self.path_start_time,
        self.movement_speed,
        self.enable_transit_integration,
    );

    if (self.path_planner) |*pp| pp.deinit();
    self.path_planner = new_pp;

    try self.render_state.snapshot.updatePathPlanner(self.alloc, self.debug_path_finding, &self.path_planner.?);

    if (self.debug_path_finding) {
        return;
    }

    if (self.path_planner) |*pp| {
        if (pp.run()) |path| {
            try self.render_state.snapshot.updatePathPlanner(self.alloc, self.debug_path_finding, pp);
            path.deinit(self.alloc);
        } else |_| {}
    }
}

pub fn registerTexture(self: *App, id: usize, tex: i32) !void {
    self.render_state.textures[id] = tex;
}

pub fn monitorWayAttribute(self: *App, k: [*]const u8, v: [*]const u8) !void {
    const k_id = self.string_table.findByPointerAddress(k);
    const v_id = self.string_table.findByPointerAddress(v);

    // FIXME: index buffers have to be updated on snapshot
    const attribute_id = try self.monitored_attributes.push(k_id, v_id);
    const k_full = self.string_table.get(k_id);
    const v_full = self.string_table.get(v_id);

    gui.pushMonitoredAttribute(attribute_id, k_full.ptr, k_full.len, v_full.ptr, v_full.len);
}

pub fn removeMonitoredAttribute(self: *App, id: usize) !void {
    try self.monitored_attributes.remove(id);

    gui.clearMonitoredAttributes();
    for (self.monitored_attributes.cost.attributes.items, 0..) |item, i| {
        const k_s = self.string_table.get(item.k);
        const v_s = self.string_table.get(item.v);
        gui.pushMonitoredAttribute(i, k_s.ptr, k_s.len, v_s.ptr, v_s.len);
    }
}

pub fn setMonitoredCostMultiplier(self: *App, id: usize, multiplier: f32) !void {
    self.monitored_attributes.cost.update(id, multiplier);
}

const ClosestWayCalculator = struct {
    // Static external references
    points: PointLookup,
    ways: WayLookup,
    potential_ways: []const WayId,
    pos: MapPos,

    // Iteration data
    way_idx: usize,
    segment_idx: usize,

    // Tracking data/output
    min_dist: f32,
    min_dist_loc: MapPos,
    min_way: WayId,
    min_way_segment: usize,
    neighbor_segment: usize,

    const DebugInfo = struct {
        dist: f32,
        dist_loc: MapPos,
    };

    fn init(p: MapPos, ways: WayLookup, potential_ways: []const WayId, points: PointLookup) ClosestWayCalculator {
        return ClosestWayCalculator{
            .ways = ways,
            .potential_ways = potential_ways,
            .way_idx = 0,
            .points = points,
            .segment_idx = 0,
            .min_dist = std.math.inf(f32),
            .min_dist_loc = undefined,
            .min_way = undefined,
            .min_way_segment = undefined,
            .neighbor_segment = undefined,
            .pos = p,
        };
    }

    fn step(self: *ClosestWayCalculator) ?DebugInfo {
        while (true) {
            if (self.way_idx >= self.potential_ways.len) {
                return null;
            }

            const way_points = self.ways.get(self.potential_ways[self.way_idx]).node_ids;

            if (self.segment_idx >= way_points.len - 1) {
                self.segment_idx = 0;
                self.way_idx += 1;

                continue;
            }

            defer self.segment_idx += 1;

            const a_point_id = way_points[self.segment_idx];
            const b_point_id = way_points[self.segment_idx + 1];

            const a = self.points.get(a_point_id);
            const b = self.points.get(b_point_id);

            const dist_loc = lin.closestPointOnLine(self.pos, a, b);
            const dist = self.pos.sub(dist_loc).length();

            const ret = DebugInfo{
                .dist = dist,
                .dist_loc = dist_loc,
            };

            if (dist < self.min_dist) {
                self.min_dist = dist;
                self.min_way = self.potential_ways[self.way_idx];
                self.min_dist_loc = ret.dist_loc;
                const ab_len = b.sub(a).length();
                const ap_len = self.pos.sub(a).length();
                if (ap_len / ab_len > 0.5) {
                    self.min_way_segment = self.segment_idx + 1;
                    self.neighbor_segment = self.segment_idx;
                } else {
                    self.min_way_segment = self.segment_idx;
                    self.neighbor_segment = self.segment_idx + 1;
                }
            }

            return ret;
        }
    }
};

const TileLocation = struct {
    center: Point,
    scale: Vec,
};

fn tileLocScreenSpace(item_metadata: image_tile_data.Item, view_state: ViewState, metadata: Metadata) TileLocation {
    const converter = map_data.CoordinateSpaceConverter.init(&metadata);
    const x_m = converter.lonToM(item_metadata.center[0]);
    const y_m = converter.latToM(item_metadata.center[1]);
    const x_s = (x_m - view_state.center.x) * view_state.zoom;
    const y_s = (y_m - view_state.center.y) * view_state.zoom * view_state.aspect;

    const w = item_metadata.size[0] / converter.width_deg * converter.widthM() * view_state.zoom / 2;
    const h = item_metadata.size[1] / converter.height_deg * converter.heightM() * view_state.zoom / 2 * view_state.aspect;

    return .{
        .center = .{
            .x = x_s,
            .y = y_s,
        },
        .scale = .{
            .x = w,
            .y = h,
        },
    };
}
