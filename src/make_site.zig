const std = @import("std");
const builtin = @import("builtin");
const Metadata = @import("Metadata.zig");
const XmlParser = @import("XmlParser.zig");
const Allocator = std.mem.Allocator;

const NodeIdIdxMap = std.AutoHashMap(i64, usize);

const StringTag = struct {
    key: []const u8,
    val: []const u8,
};

const WayCache = struct {
    alloc: Allocator,
    found_highway: bool = false,
    tags: std.ArrayList(StringTag),
    nodes: std.ArrayList(i64),

    fn deinit(self: *WayCache) void {
        self.nodes.deinit();
        for (self.tags.items) |tag| {
            self.alloc.free(tag.key);
            self.alloc.free(tag.val);
        }
        self.tags.deinit();
    }

    fn pushNode(self: *WayCache, node_id: i64) !void {
        try self.nodes.append(node_id);
    }

    fn pushTag(self: *WayCache, k: []const u8, v: []const u8) !void {
        if (std.mem.eql(u8, k, "highway")) {
            self.found_highway = true;
        }

        try self.tags.append(.{
            .key = try self.alloc.dupe(u8, k),
            .val = try self.alloc.dupe(u8, v),
        });
    }

    fn reset(self: *WayCache) void {
        self.found_highway = false;
        for (self.tags.items) |tag| {
            self.alloc.free(tag.key);
            self.alloc.free(tag.val);
        }
        self.tags.clearRetainingCapacity();
        self.nodes.clearRetainingCapacity();
    }
};

const MapDataWriter = struct {
    writer: std.io.AnyWriter,
    node_id_idx_map: NodeIdIdxMap,
    min_lat: f32 = std.math.floatMax(f32),
    max_lat: f32 = -std.math.floatMax(f32),
    min_lon: f32 = std.math.floatMax(f32),
    max_lon: f32 = -std.math.floatMax(f32),

    fn deinit(self: *MapDataWriter) void {
        self.node_id_idx_map.deinit();
    }

    fn pushNode(self: *MapDataWriter, node_id: i64, lon: f32, lat: f32) !void {
        self.max_lon = @max(lon, self.max_lon);
        self.min_lon = @min(lon, self.min_lon);
        self.max_lat = @max(lat, self.max_lat);
        self.min_lat = @min(lat, self.min_lat);

        self.node_id_idx_map.put(node_id, self.node_id_idx_map.count()) catch return;

        comptime std.debug.assert(builtin.cpu.arch.endian() == .little);
        try self.writer.writeAll(std.mem.asBytes(&lon));
        try self.writer.writeAll(std.mem.asBytes(&lat));
    }

    fn pushWayNodes(self: *MapDataWriter, nodes: []const i64) !void {
        try self.writer.writeInt(u32, 0xffffffff, .little);
        for (nodes) |node_id| {
            const node_idx = self.node_id_idx_map.get(node_id) orelse return error.NoNode;
            try self.writer.writeInt(u32, @intCast(node_idx), .little);
        }
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
    string_table: std.StringHashMap(usize),
    string_table_data: std.ArrayList([]const u8),

    fn init(alloc: Allocator) StringTable {
        return .{
            .alloc = alloc,
            .string_table = std.StringHashMap(usize).init(alloc),
            .string_table_data = std.ArrayList([]const u8).init(alloc),
        };
    }

    fn deinit(self: *StringTable) void {
        for (self.string_table_data.items) |item| {
            self.alloc.free(item);
        }

        self.string_table.deinit();
        self.string_table_data.deinit();
    }

    fn pushOrFree(self: *StringTable, s: []const u8) !usize {
        const res = try self.string_table.getOrPut(s);
        if (!res.found_existing) {
            try self.string_table_data.append(s);
            res.value_ptr.* = self.string_table_data.items.len - 1;
        } else {
            self.alloc.free(s);
        }

        return res.value_ptr.*;
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
        user_data.way_cache.reset();
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

            try user_data.way_cache.pushTag(k, v);
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
                const k = try user_data.string_table.pushOrFree(tag.key);
                const v = try user_data.string_table.pushOrFree(tag.val);

                this_way_tag_keys[i] = k;
                this_way_tag_vals[i] = v;
            }

            try user_data.way_tags.append(.{
                this_way_tag_keys,
                this_way_tag_vals,
            });
            user_data.way_cache.tags.clearRetainingCapacity();
            user_data.way_cache.reset();
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
    it: std.process.ArgIterator,

    const Option = enum {
        @"--osm-data",
        @"--input-www",
        @"--index-wasm",
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
                .@"--output" => output_opt = it.next(),
            }
        }

        return .{
            .osm_data = osm_data_opt orelse return error.NoOsmData,
            .input_www = input_www_opt orelse return error.NoWww,
            .index_wasm = index_wasm_opt orelse return error.NoWasm,
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

    try runParser(alloc, args.osm_data, .{
        .ctx = &userdata,
        .startElement = startElement,
        .endElement = endElement,
    });

    var data_writer = MapDataWriter{
        .node_id_idx_map = NodeIdIdxMap.init(alloc),
        .writer = counting_writer.writer().any(),
    };
    defer data_writer.deinit();

    var seen_node_ids = std.AutoHashMap(i64, void).init(alloc);
    defer seen_node_ids.deinit();

    for (userdata.way_nodes.items) |way_nodes| {
        for (way_nodes) |node_id| {
            const seen = try seen_node_ids.getOrPut(node_id);
            if (!seen.found_existing) {
                const node: NodeData = userdata.node_storage.get(node_id) orelse return error.NoNode;
                try data_writer.pushNode(node_id, node.lon, node.lat);
            }
        }
    }

    for (userdata.way_nodes.items) |way_nodes| {
        try data_writer.pushWayNodes(way_nodes);
    }

    const end_ways = counting_writer.bytes_written;

    for (userdata.string_table.string_table_data.items) |item| {
        try data_writer.pushStringTableString(item);
    }

    const metadata_out_f = try std.fs.cwd().createFile(metadata_path, .{});
    const metadata = Metadata{
        .min_lat = data_writer.min_lat,
        .max_lat = data_writer.max_lat,
        .min_lon = data_writer.min_lon,
        .max_lon = data_writer.max_lon,
        .end_nodes = data_writer.node_id_idx_map.count() * 8,
        .end_ways = end_ways,
        .way_tags = try userdata.way_tags.toOwnedSlice(),
    };
    try std.json.stringify(metadata, .{}, metadata_out_f.writer());

    for (metadata.way_tags) |way_tags| {
        alloc.free(way_tags[0]);
        alloc.free(way_tags[1]);
    }
    alloc.free(metadata.way_tags);
}
