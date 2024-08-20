const std = @import("std");
const builtin = @import("builtin");
const Metadata = @import("Metadata.zig");
const XmlParser = @import("XmlParser.zig");
const image_tile_data_mod = @import("image_tile_data.zig");
const Allocator = std.mem.Allocator;
const TransitDb = @import("TransitDb.zig");
const map_data = @import("map_data.zig");

const NodeIdIdxMap = std.AutoHashMap(i64, usize);

const StringTag = struct {
    key: usize,
    val: usize,
};

const WayCache = struct {
    alloc: Allocator,
    found_highway: bool = false,
    tags: std.ArrayList(StringTag),
    nodes: std.ArrayList(i64),
    last_st_size: usize = 0,

    fn deinit(self: *WayCache) void {
        self.nodes.deinit();
        self.tags.deinit();
    }

    fn pushNode(self: *WayCache, node_id: i64) !void {
        try self.nodes.append(node_id);
    }

    fn pushTag(self: *WayCache, k: []const u8, v: []const u8, st: *StringTable) !void {
        if (std.mem.eql(u8, k, "highway")) {
            self.found_highway = true;
        }

        try self.tags.append(.{
            .key = try st.push(k),
            .val = try st.push(v),
        });
    }

    fn reset(self: *WayCache, string_table_size: usize) void {
        self.found_highway = false;
        self.last_st_size = string_table_size;
        self.tags.clearRetainingCapacity();
        self.nodes.clearRetainingCapacity();
    }
};

const MapDataWriter = struct {
    writer: std.io.AnyWriter,
    osm_nodes: NodeIdIdxMap,
    gtfs_nodes: NodeIdIdxMap,
    num_pushed_nodes: usize = 0,
    num_pushed_ways: usize = 0,
    min_lat: f32 = std.math.floatMax(f32),
    max_lat: f32 = -std.math.floatMax(f32),
    min_lon: f32 = std.math.floatMax(f32),
    max_lon: f32 = -std.math.floatMax(f32),

    fn deinit(self: *MapDataWriter) void {
        self.osm_nodes.deinit();
        self.gtfs_nodes.deinit();
    }

    fn pushOsmNode(self: *MapDataWriter, node_id: i64, lon: f32, lat: f32) !void {
        const idx = try self.pushNodeImpl(lon, lat);
        try self.osm_nodes.put(node_id, idx);
    }

    fn pushGtfsNode(self: *MapDataWriter, node_id: i64, lon: f32, lat: f32) !void {
        const idx = try self.pushNodeImpl(lon, lat);
        try self.gtfs_nodes.put(node_id, idx);
    }

    fn pushNodeImpl(self: *MapDataWriter, lon: f32, lat: f32) !usize {
        self.max_lon = @max(lon, self.max_lon);
        self.min_lon = @min(lon, self.min_lon);
        self.max_lat = @max(lat, self.max_lat);
        self.min_lat = @min(lat, self.min_lat);

        comptime std.debug.assert(builtin.cpu.arch.endian() == .little);
        try self.writer.writeAll(std.mem.asBytes(&lon));
        try self.writer.writeAll(std.mem.asBytes(&lat));

        defer self.num_pushed_nodes += 1;
        return self.num_pushed_nodes;
    }

    fn pushOsmWay(self: *MapDataWriter, nodes: []const i64) !void {
        try self.writer.writeInt(u32, 0xffffffff, .little);
        for (nodes) |node_id| {
            const node_idx = self.osm_nodes.get(node_id) orelse return error.NoNode;
            try self.writer.writeInt(u32, @intCast(node_idx), .little);
        }

        self.num_pushed_ways += 1;
    }

    fn pushGtfsWay(self: *MapDataWriter, nodes: []const i64) !void {
        try self.writer.writeInt(u32, 0xffffffff, .little);
        for (nodes) |node_id| {
            const node_idx = self.gtfs_nodes.get(node_id) orelse return error.NoNode;
            try self.writer.writeInt(u32, @intCast(node_idx), .little);
        }

        self.num_pushed_ways += 1;
    }

    fn pushWayByIndex(self: *MapDataWriter, nodes: []const usize) !void {
        try self.writer.writeInt(u32, 0xffffffff, .little);
        for (nodes) |node_idx| {
            try self.writer.writeInt(u32, @intCast(node_idx), .little);
        }

        self.num_pushed_ways += 1;
    }

    fn pushStringTableString(self: *MapDataWriter, s: []const u8) !void {
        comptime std.debug.assert(builtin.cpu.arch.endian() == .little);
        try self.writer.writeInt(u16, @intCast(s.len), .little);
        try self.writer.writeAll(s);
    }
};

const NodeData = struct {
    lon: f32,
    lat: f32,
};

const StringTable = struct {
    alloc: Allocator,
    inner: std.StringArrayHashMapUnmanaged(usize) = .{},

    fn init(alloc: Allocator) StringTable {
        return .{ .alloc = alloc };
    }

    fn deinit(self: *StringTable) void {
        for (self.inner.keys()) |key| {
            self.alloc.free(key);
        }

        self.inner.deinit(self.alloc);
    }

    fn push(self: *StringTable, str: []const u8) !usize {
        const count_ = self.inner.count();
        const gop = try self.inner.getOrPut(self.alloc, str);
        if (!gop.found_existing) {
            gop.key_ptr.* = try self.alloc.dupe(u8, str);
            gop.value_ptr.* = count_;
        }
        return gop.value_ptr.*;
    }

    fn count(self: *const StringTable) usize {
        return self.inner.count();
    }

    fn rollback(self: *StringTable, size: usize) void {
        // clear any extra string table entries we may have added
        for (self.inner.keys()[size..]) |key| {
            self.alloc.free(key);
        }
        self.inner.shrinkRetainingCapacity(size);
    }
};

const StringTableIndex = usize;
const Userdata = struct {
    alloc: Allocator,
    way_tags: std.ArrayList(Metadata.Tags),
    in_way: bool = false,
    node_storage: std.AutoHashMap(i64, NodeData),
    way_nodes: std.ArrayList([]i64),
    way_cache: WayCache,
    string_table: StringTable,

    fn deinit(self: *Userdata) void {
        self.string_table.deinit();
        self.way_cache.deinit();
        for (self.way_tags.items) |item| {
            self.alloc.free(item[0]);
            self.alloc.free(item[1]);
        }
        self.way_tags.deinit();
        self.node_storage.deinit();
        for (self.way_nodes.items) |item| {
            self.alloc.free(item);
        }
        self.way_nodes.deinit();
    }

    fn handleNode(user_data: *Userdata, attrs: *XmlParser.XmlAttrIter) void {
        var lat_opt: ?[]const u8 = null;
        var lon_opt: ?[]const u8 = null;
        var node_id_opt: ?[]const u8 = null;
        while (attrs.next()) |attr| {
            if (std.mem.eql(u8, attr.key, "lat")) {
                lat_opt = attr.val;
            } else if (std.mem.eql(u8, attr.key, "lon")) {
                lon_opt = attr.val;
            } else if (std.mem.eql(u8, attr.key, "id")) {
                node_id_opt = attr.val;
            }
        }

        const lat_s = lat_opt orelse return;
        const lon_s = lon_opt orelse return;
        const node_id_s = node_id_opt orelse return;
        const lat = std.fmt.parseFloat(f32, lat_s) catch return;
        const lon = std.fmt.parseFloat(f32, lon_s) catch return;
        const node_id = std.fmt.parseInt(i64, node_id_s, 10) catch return;
        user_data.node_storage.put(node_id, .{
            .lon = lon,
            .lat = lat,
        }) catch return;
    }
};

fn findAttributeVal(key: []const u8, attrs: *XmlParser.XmlAttrIter) ?[]const u8 {
    while (attrs.next()) |attr| {
        if (std.mem.eql(u8, attr.key, key)) {
            return attr.val;
        }
    }

    return null;
}

fn startElement(ctx: ?*anyopaque, name: []const u8, attrs: *XmlParser.XmlAttrIter) anyerror!void {
    const user_data: *Userdata = @ptrCast(@alignCast(ctx));

    if (std.mem.eql(u8, name, "node")) {
        user_data.handleNode(attrs);
        return;
    } else if (std.mem.eql(u8, name, "way")) {
        user_data.in_way = true;
        user_data.string_table.rollback(user_data.way_cache.last_st_size);
        user_data.way_cache.reset(user_data.string_table.count());
    } else if (std.mem.eql(u8, name, "nd")) {
        if (user_data.in_way) {
            const node_id_s = findAttributeVal("ref", attrs) orelse return error.NoRef;
            const node_id = try std.fmt.parseInt(i64, node_id_s, 10);
            try user_data.way_cache.pushNode(node_id);
        }
    } else if (std.mem.eql(u8, name, "tag")) {
        if (user_data.in_way) {
            var k_opt: ?[]const u8 = null;
            var v_opt: ?[]const u8 = null;
            while (attrs.next()) |attr| {
                if (std.mem.eql(u8, attr.key, "k")) {
                    k_opt = attr.val;
                } else if (std.mem.eql(u8, attr.key, "v")) {
                    v_opt = attr.val;
                }
            }

            const k = k_opt orelse return error.NoTagKey;
            const v = v_opt orelse return error.NoValKey;

            try user_data.way_cache.pushTag(k, v, &user_data.string_table);
        }
    }
}

fn endElement(ctx: ?*anyopaque, name: []const u8) anyerror!void {
    const user_data: *Userdata = @ptrCast(@alignCast(ctx));
    if (std.mem.eql(u8, name, "way")) {
        user_data.in_way = false;
        if (user_data.way_cache.found_highway) {
            try user_data.way_nodes.append(try user_data.way_cache.nodes.toOwnedSlice());
            var this_way_tag_keys = try user_data.alloc.alloc(usize, user_data.way_cache.tags.items.len);
            var this_way_tag_vals = try user_data.alloc.alloc(usize, user_data.way_cache.tags.items.len);

            for (user_data.way_cache.tags.items, 0..) |tag, i| {
                this_way_tag_keys[i] = tag.key;
                this_way_tag_vals[i] = tag.val;
            }

            try user_data.way_tags.append(.{
                this_way_tag_keys,
                this_way_tag_vals,
            });
            user_data.way_cache.reset(user_data.string_table.count());
        } else {
            user_data.string_table.rollback(user_data.way_cache.last_st_size);
            user_data.way_cache.reset(user_data.string_table.count());
        }
    }
}

fn runParser(alloc: Allocator, xml_path: []const u8, callbacks: XmlParser.Callbacks) !void {
    const f = try std.fs.cwd().openFile(xml_path, .{});
    defer f.close();

    var buffered_reader = std.io.bufferedReader(f.reader());

    var parser = try XmlParser.init(alloc, callbacks);
    defer parser.deinit();

    while (true) {
        var buf: [4096]u8 = undefined;
        const read_data_len = try buffered_reader.read(&buf);
        if (read_data_len == 0) {
            try parser.finish();
            break;
        }

        try parser.feed(buf[0..read_data_len]);
    }
}

const Args = struct {
    osm_data: []const u8,
    input_www: []const u8,
    index_wasm: []const u8,
    output: []const u8,
    image_tile_data: []const u8,
    gtfs_data: []const u8,
    business_data: ?[]const u8,
    it: std.process.ArgIterator,

    const Option = enum {
        @"--osm-data",
        @"--input-www",
        @"--index-wasm",
        @"--image-tile-data",
        @"--gtfs-data",
        @"--business-data",
        @"--output",
    };
    fn deinit(self: *Args) void {
        self.it.deinit();
    }

    fn parse(alloc: Allocator) !Args {
        var it = try std.process.ArgIterator.initWithAllocator(alloc);
        _ = it.next();

        var osm_data_opt: ?[]const u8 = null;
        var input_www_opt: ?[]const u8 = null;
        var index_wasm_opt: ?[]const u8 = null;
        var image_tile_data: []const u8 = &.{};
        var gtfs_data: ?[]const u8 = null;
        var business_data: ?[]const u8 = null;
        var output_opt: ?[]const u8 = null;

        while (it.next()) |arg| {
            const opt = std.meta.stringToEnum(Option, arg) orelse {
                std.debug.print("{s}", .{arg});
                return error.InvalidOption;
            };

            switch (opt) {
                .@"--osm-data" => osm_data_opt = it.next(),
                .@"--input-www" => input_www_opt = it.next(),
                .@"--index-wasm" => index_wasm_opt = it.next(),
                .@"--image-tile-data" => image_tile_data = it.next() orelse @panic("no --image-tile arg"),
                .@"--gtfs-data" => gtfs_data = it.next() orelse @panic("no --gtfs-data arg"),
                .@"--business-data" => business_data = it.next() orelse @panic("no --business-data arg"),
                .@"--output" => output_opt = it.next(),
            }
        }

        return .{
            .osm_data = osm_data_opt orelse return error.NoOsmData,
            .input_www = input_www_opt orelse return error.NoWww,
            .index_wasm = index_wasm_opt orelse return error.NoWasm,
            .image_tile_data = image_tile_data,
            .gtfs_data = gtfs_data orelse return error.NoGtfs,
            .business_data = business_data,
            .output = output_opt orelse return error.NoOutput,
            .it = it,
        };
    }
};

fn linkFile(alloc: Allocator, in: []const u8, out_dir: []const u8) !void {
    const name = std.fs.path.basename(in);
    const link_path = try std.fs.path.relative(alloc, out_dir, in);
    defer alloc.free(link_path);

    const out = try std.fs.cwd().openDir(out_dir, .{});
    out.deleteFile(name) catch {};
    try out.symLink(link_path, name, std.fs.Dir.SymLinkFlags{});
}

fn linkWww(alloc: Allocator, in: []const u8, out_dir: []const u8) !void {
    const in_dir = try std.fs.cwd().openDir(in, .{
        .iterate = true,
    });

    var it = in_dir.iterate();
    while (try it.next()) |entry| {
        const p = try std.fs.path.join(alloc, &.{ in, entry.name });
        defer alloc.free(p);

        try linkFile(alloc, p, out_dir);
    }
}

const BusinessInfo = struct {
    points: []f32 = &.{},
    // String table index
    names: []u32 = &.{},

    fn deinit(self: *BusinessInfo, alloc: Allocator) void {
        alloc.free(self.points);
        alloc.free(self.names);
    }
};

fn processBusinesses(alloc: Allocator, business_data_path: []const u8, st: *StringTable) !BusinessInfo {
    const Business = struct {
        name: []const u8,
        lon: f32,
        lat: f32,
    };

    const business_f = try std.fs.cwd().openFile(business_data_path, .{});
    var json_reader = std.json.reader(alloc, business_f.reader());
    defer json_reader.deinit();

    var diagnostics: std.json.Diagnostics = .{};
    json_reader.enableDiagnostics(&diagnostics);

    const business_data = std.json.parseFromTokenSource([]Business, alloc, &json_reader, .{}) catch |e| {
        std.log.err("{s} @ {d}:{d}", .{ @errorName(e), diagnostics.getLine(), diagnostics.getColumn() });
        return e;
    };
    defer business_data.deinit();

    const points = try alloc.alloc(f32, business_data.value.len * 2);
    errdefer alloc.free(points);

    const names = try alloc.alloc(u32, business_data.value.len);
    errdefer alloc.free(names);

    for (business_data.value, 0..) |item, i| {
        names[i] = @intCast(try st.push(item.name));
        points[i * 2] = item.lon;
        points[i * 2 + 1] = item.lat;
    }

    return .{
        .points = points,
        .names = names,
    };
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();

    const alloc = gpa.allocator();

    var args = try Args.parse(alloc);
    defer args.deinit();

    try std.fs.cwd().makePath(args.output);

    try linkFile(alloc, args.index_wasm, args.output);
    try linkWww(alloc, args.input_www, args.output);

    const map_data_path = try std.fs.path.join(alloc, &.{ args.output, "map_data.bin" });
    defer alloc.free(map_data_path);

    const metadata_path = try std.fs.path.join(alloc, &.{ args.output, "map_data.json" });
    defer alloc.free(metadata_path);

    const out_f = try std.fs.cwd().createFile(map_data_path, .{});
    defer out_f.close();

    var points_out_buf_writer = std.io.bufferedWriter(out_f.writer());
    defer points_out_buf_writer.flush() catch |e| {
        std.log.err("Failed to flush: {any}", .{e});
    };
    var counting_writer = std.io.countingWriter(points_out_buf_writer.writer());

    var userdata = Userdata{
        .alloc = alloc,
        .way_cache = .{
            .alloc = alloc,
            .tags = std.ArrayList(StringTag).init(alloc),
            .nodes = std.ArrayList(i64).init(alloc),
        },
        .way_tags = std.ArrayList(Metadata.Tags).init(alloc),
        .node_storage = std.AutoHashMap(i64, NodeData).init(alloc),
        .way_nodes = std.ArrayList([]i64).init(alloc),
        .string_table = StringTable.init(alloc),
    };
    defer userdata.deinit();

    var transit_db = try TransitDb.init(alloc, args.gtfs_data);
    var stops = try transit_db.getStops(alloc);
    defer stops.deinit(alloc);

    const trips = try transit_db.getTrips(alloc);
    defer {
        for (trips) |*trip| trip.deinit(alloc);
        alloc.free(trips);
    }

    var trip_stop_times = std.ArrayList([]u32).init(alloc);
    defer trip_stop_times.deinit();

    var routes = try transit_db.getRoutes(alloc);
    defer routes.deinit(alloc);

    try runParser(alloc, args.osm_data, .{
        .ctx = &userdata,
        .startElement = startElement,
        .endElement = endElement,
    });

    const processed_businesses: BusinessInfo = if (args.business_data) |p| try processBusinesses(alloc, p, &userdata.string_table) else .{};

    var data_writer = MapDataWriter{
        .osm_nodes = NodeIdIdxMap.init(alloc),
        .gtfs_nodes = NodeIdIdxMap.init(alloc),
        .writer = counting_writer.writer().any(),
    };
    defer data_writer.deinit();

    var seen_osm_nodes = std.AutoHashMap(i64, void).init(alloc);
    defer seen_osm_nodes.deinit();

    for (userdata.way_nodes.items) |way_nodes| {
        for (way_nodes) |node_id| {
            const seen = try seen_osm_nodes.getOrPut(node_id);
            if (!seen.found_existing) {
                const node: NodeData = userdata.node_storage.get(node_id) orelse return error.NoNode;
                try data_writer.pushOsmNode(node_id, node.lon, node.lat);
            }
        }
    }

    const transit_node_start: u32 = @intCast(data_writer.num_pushed_nodes);

    for (0..stops.ids.len) |i| {
        const stop_id = stops.ids[i];
        const stop_loc = stops.points[i];
        try data_writer.pushGtfsNode(stop_id, stop_loc.lon, stop_loc.lat);
    }

    const end_nodes = counting_writer.bytes_written;

    for (userdata.way_nodes.items) |way_nodes| {
        try data_writer.pushOsmWay(way_nodes);
    }

    const transit_way_start: u32 = @intCast(data_writer.num_pushed_ways);

    const short_name_key = try userdata.string_table.push("short_name");
    const long_name_key = try userdata.string_table.push("long_name");

    for (trips) |trip| {
        const route_id = routes.trip_to_route_idx.get(trip.trip_id).?;
        const short_name = routes.route_short_names[route_id];
        const long_name = routes.route_long_names[route_id];

        const short_name_val = try userdata.string_table.push(short_name);
        const long_name_val = try userdata.string_table.push(long_name);

        try data_writer.pushGtfsWay(trip.stop_ids);
        try trip_stop_times.append(trip.stop_times);

        const keys = try alloc.dupe(usize, &.{ short_name_key, long_name_key });
        errdefer alloc.free(keys);

        const vals = try alloc.dupe(usize, &.{ short_name_val, long_name_val });
        errdefer alloc.free(vals);

        try userdata.way_tags.append(Metadata.Tags{ keys, vals });
    }

    const osm_to_transit_way_start: u32 = @intCast(data_writer.num_pushed_ways);
    const width = data_writer.max_lon - data_writer.min_lon;
    const height = data_writer.max_lat - data_writer.min_lat;

    var osm_node_buckets_builder = try map_data.MapBuckets(i64).Builder.init(alloc, width, height);
    errdefer osm_node_buckets_builder.deinit();

    var osm_node_it = data_writer.osm_nodes.keyIterator();
    while (osm_node_it.next()) |osm_id| {
        const data = userdata.node_storage.get(osm_id.*).?;
        try osm_node_buckets_builder.push(osm_id.*, data.lat - data_writer.min_lat, data.lon - data_writer.min_lon);
    }

    var osm_node_buckets = try osm_node_buckets_builder.build();
    defer osm_node_buckets.deinit(alloc);

    // NOTE: Lat/lon space does not result in real closest point
    // NOTE: OSM may not have a close point that is reasonable to board the bus from :)
    for (0..stops.ids.len) |i| {
        const stop_id = stops.ids[i];
        const stop_loc = stops.points[i];
        var closest_node: i64 = undefined;
        var closest_dist = std.math.inf(f32);

        const close_node_ids = osm_node_buckets.get(stop_loc.lat - data_writer.min_lat, stop_loc.lon - data_writer.min_lon);
        for (close_node_ids) |osm_id| {
            const osm_point = userdata.node_storage.get(osm_id).?;
            const lat_dist = osm_point.lat - stop_loc.lat;
            const lon_dist = osm_point.lon - stop_loc.lon;

            const total_dist_2 = lat_dist * lat_dist + lon_dist * lon_dist;
            if (total_dist_2 < closest_dist) {
                closest_node = osm_id;
                closest_dist = total_dist_2;
            }
        }
        if (closest_dist != std.math.inf(f32)) {
            const transit_idx = data_writer.gtfs_nodes.get(stop_id).?;
            const osm_idx = data_writer.osm_nodes.get(closest_node).?;
            try data_writer.pushWayByIndex(&.{ osm_idx, transit_idx });
            try userdata.way_tags.append(Metadata.Tags{ &.{}, &.{} });
        }
    }

    const end_ways = counting_writer.bytes_written;

    try data_writer.writer.writeAll(std.mem.sliceAsBytes(processed_businesses.points));

    const end_businesses = counting_writer.bytes_written;

    for (userdata.string_table.inner.keys()) |key| {
        try data_writer.pushStringTableString(key);
    }

    const metadata_out_f = try std.fs.cwd().createFile(metadata_path, .{});
    const metadata = Metadata{
        .min_lat = data_writer.min_lat,
        .max_lat = data_writer.max_lat,
        .min_lon = data_writer.min_lon,
        .max_lon = data_writer.max_lon,
        .end_nodes = end_nodes,
        .end_ways = end_ways,
        .end_businesses = end_businesses,
        .transit_node_start_idx = transit_node_start,
        .transit_way_start_idx = transit_way_start,
        .osm_to_transit_way_start_idx = osm_to_transit_way_start,
        .way_tags = try userdata.way_tags.toOwnedSlice(),
        .business_names = processed_businesses.names,
        .transit_trip_times = trip_stop_times.items,
    };
    try std.json.stringify(metadata, .{}, metadata_out_f.writer());

    for (metadata.way_tags) |way_tags| {
        alloc.free(way_tags[0]);
        alloc.free(way_tags[1]);
    }
    alloc.free(metadata.way_tags);

    const image_tile_data_path = try std.fs.path.join(alloc, &.{ args.output, "image_tile_data.json" });
    defer alloc.free(image_tile_data_path);
    if (args.image_tile_data.len == 0) {
        var image_tile_f = try std.fs.cwd().createFile(image_tile_data_path, .{
            .truncate = true,
        });
        defer image_tile_f.close();

        try image_tile_f.writeAll("[]");
    } else {
        const image_tile_f = try std.fs.cwd().openFile(args.image_tile_data, .{});
        var json_reader = std.json.reader(alloc, image_tile_f.reader());
        defer json_reader.deinit();

        const image_tile_data = try std.json.parseFromTokenSource(image_tile_data_mod.ImageTileData, alloc, &json_reader, .{});
        defer image_tile_data.deinit();

        const image_tile_data_dir = std.fs.path.dirname(args.image_tile_data) orelse @panic("no dir");
        for (image_tile_data.value) |item| {
            const input_img_path = try std.fs.path.join(alloc, &.{ image_tile_data_dir, item.path });
            std.debug.print("{s}\n", .{input_img_path});
            defer alloc.free(input_img_path);

            const output_img_path = try std.fs.path.join(alloc, &.{ args.output, item.path });
            defer alloc.free(output_img_path);

            const output_img_dir = std.fs.path.dirname(output_img_path) orelse @panic("no dir");

            try std.fs.cwd().makePath(output_img_dir);

            const link_path = try std.fs.path.relative(alloc, output_img_dir, input_img_path);
            defer alloc.free(link_path);
            std.fs.cwd().deleteFile(output_img_path) catch {};
            try std.fs.cwd().symLink(link_path, output_img_path, std.fs.Dir.SymLinkFlags{});
        }

        std.fs.cwd().deleteFile(image_tile_data_path) catch {};
        const link_path = try std.fs.path.relative(alloc, args.output, args.image_tile_data);
        defer alloc.free(link_path);
        try std.fs.cwd().symLink(link_path, image_tile_data_path, std.fs.Dir.SymLinkFlags{});
    }
}
