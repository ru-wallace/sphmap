const std = @import("std");
const builtin = @import("builtin");
const Metadata = @import("Metadata.zig");
const XmlParser = @import("XmlParser.zig");
const Allocator = std.mem.Allocator;

const Userdata = struct {
    points_out: std.io.AnyWriter,
    log: std.io.AnyWriter,
    metadata: Metadata = .{},
    node_id_idx_map: std.AutoHashMap(i64, usize),
    num_nodes: u64 = 0,
    num_refs: u64 = 0,
    num_ways: u64 = 0,
    way_id_colour_map: std.AutoHashMap(i64, [4]f32),

    stored_ways: std.ArrayList(i64),
    stored_colour: [4]f32 = .{ 1.0, 1.0, 1.0, 1.0 },

    fn deinit(self: *Userdata) void {
        self.node_id_idx_map.deinit();
        self.way_id_colour_map.deinit();
        self.stored_ways.deinit();
    }

    fn handleNode(user_data: *Userdata, attrs: *XmlParser.XmlAttrIter) void {
        defer user_data.num_nodes += 1;

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
        user_data.node_id_idx_map.put(node_id, user_data.num_nodes) catch return;

        user_data.metadata.max_lon = @max(lon, user_data.metadata.max_lon);
        user_data.metadata.min_lon = @min(lon, user_data.metadata.min_lon);
        user_data.metadata.max_lat = @max(lat, user_data.metadata.max_lat);
        user_data.metadata.min_lat = @min(lat, user_data.metadata.min_lat);

        std.debug.assert(builtin.cpu.arch.endian() == .little);
        user_data.points_out.writeAll(std.mem.asBytes(&lon)) catch unreachable;
        user_data.points_out.writeAll(std.mem.asBytes(&lat)) catch unreachable;
    }

    fn handleWay(user_data: *Userdata, attrs: *XmlParser.XmlAttrIter) void {
        defer user_data.num_refs += 1;
        defer user_data.num_ways += 1;

        user_data.points_out.writeInt(u32, 0xffffffff, .little) catch unreachable;

        var way_id_opt: ?[]const u8 = null;
        while (attrs.next()) |attr| {
            if (std.mem.eql(u8, attr.key, "id")) {
                way_id_opt = attr.val;
            }
        }

        const way_id_s = way_id_opt orelse return;
        const way_id = std.fmt.parseInt(i64, way_id_s, 10) catch return;
        user_data.way_id_colour_map.put(way_id, .{ 1.0, 1.0, 1.0, 1.0 }) catch unreachable;
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

fn getColour(user_data: *Userdata, attrs: *XmlParser.XmlAttrIter) [4]f32 {
    var colour: [4]f32 = .{ 1.0, 1.0, 1.0, 1.0 };

    const colourAttr = findAttributeVal("v", attrs) orelse return colour;

    if (colourAttr[0] == '#') {
        const r_channel: f32 = @as(f32, @floatFromInt(std.fmt.parseInt(i32, colourAttr[1..3], 16) catch 255));
        const g_channel: f32 = @as(f32, @floatFromInt(std.fmt.parseInt(i32, colourAttr[3..5], 16) catch 255));
        const b_channel: f32 = @as(f32, @floatFromInt(std.fmt.parseInt(i32, colourAttr[5..7], 16) catch 255));
        var alpha_channel: f32 = 255;
        if (colourAttr.len == 9) {
            alpha_channel = @as(f32, @floatFromInt(std.fmt.parseInt(i32, colourAttr[7..9], 16) catch 255));
        }

        colour = .{
            r_channel / 255,
            g_channel / 255,
            b_channel / 255,
            alpha_channel / 255,
        };
    }

    if (std.mem.eql(u8, colourAttr, "red")) {
        colour = .{ 1.0, 0.0, 0.0, 1.0 };
    } else if (std.mem.eql(u8, colourAttr, "green")) {
        colour = .{ 0.0, 1.0, 0.0, 1.0 };
    } else if (std.mem.eql(u8, colourAttr, "blue")) {
        colour = .{ 0.0, 0.0, 1.0, 1.0 };
    }

    user_data.log.print("Colour attr: {s} => ({d}, {d}, {d}, {d})\n", .{ colourAttr, colour[0] * 255, colour[1] * 255, colour[2] * 255, colour[3] * 255 }) catch |err| {
        _ = user_data.log.print("Error printing: {}\n", .{err}) catch null;
    };

    return colour;
}

fn startElement(ctx: ?*anyopaque, name: []const u8, attrs: *XmlParser.XmlAttrIter) void {
    const user_data: *Userdata = @ptrCast(@alignCast(ctx));

    if (std.mem.eql(u8, name, "node")) {
        user_data.handleNode(attrs);
        return;
    } else if (std.mem.eql(u8, name, "way")) {
        user_data.handleWay(attrs);
        return;
    } else if (std.mem.eql(u8, name, "nd")) {
        user_data.num_refs += 1;
        const node_id_s = findAttributeVal("ref", attrs) orelse return;

        const node_id = std.fmt.parseInt(i64, node_id_s, 10) catch unreachable;
        const node_idx = user_data.node_id_idx_map.get(node_id) orelse return;
        user_data.points_out.writeInt(u32, @intCast(node_idx), .little) catch unreachable;
    } else if (std.mem.eql(u8, name, "member")) {
        const member_type = findAttributeVal("type", attrs) orelse return;
        if (std.mem.eql(u8, member_type, "way")) {
            const way_id_s = findAttributeVal("ref", attrs) orelse return;
            const way_id = std.fmt.parseInt(i64, way_id_s, 10) catch unreachable;
            //user_data.log.print("Found way {}\n", .{way_id}) catch {};
            user_data.stored_ways.append(way_id) catch {};
        }
    } else if (std.mem.eql(u8, name, "tag")) {
        const tagName = findAttributeVal("k", attrs) orelse return;
        if (std.mem.eql(u8, tagName, "colour") or std.mem.eql(u8, tagName, "color")) {
            const colour = getColour(user_data, attrs);
            user_data.stored_colour = colour;
        }
    }
}

fn endElement(ctx: ?*anyopaque, name: []const u8) void {
    const user_data: *Userdata = @ptrCast(@alignCast(ctx));

    if (std.mem.eql(u8, name, "relation")) {
        //user_data.log.print("Relation\n", .{}) catch {};
        const white: [4]f32 = .{ 1.0, 1.0, 1.0, 1.0 };
        if (!std.mem.eql(f32, &user_data.stored_colour, &white)) {
            const stored_ways = user_data.stored_ways.toOwnedSlice() catch return;
            for (stored_ways) |way_id| {
                //user_data.log.print("Stored Way: {}, colour: {any}\n", .{ way_id, user_data.stored_colour }) catch {};

                user_data.way_id_colour_map.put(way_id, user_data.stored_colour) catch unreachable;
            }
            user_data.stored_ways.allocator.free(stored_ways);
        }
        user_data.stored_colour = white;
    }
}

fn runParser(xml_path: []const u8, callbacks: XmlParser.Callbacks) !void {
    const f = try std.fs.cwd().openFile(xml_path, .{});
    defer f.close();

    var buffered_reader = std.io.bufferedReader(f.reader());

    var parser = try XmlParser.init(&callbacks);
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
    var points_out_buf_writer = std.io.bufferedWriter(out_f.writer());
    defer points_out_buf_writer.flush() catch unreachable;
    const points_out_writer = points_out_buf_writer.writer().any();

    const log_f = try std.fs.cwd().createFile("make_site.log", .{});
    var log_buf_writer = std.io.bufferedWriter(log_f.writer());
    defer log_buf_writer.flush() catch unreachable;
    const log_writer = log_buf_writer.writer().any();

    var userdata = Userdata{
        .points_out = points_out_writer,
        .log = log_writer,
        .node_id_idx_map = std.AutoHashMap(i64, usize).init(alloc),
        .way_id_colour_map = std.AutoHashMap(i64, [4]f32).init(alloc),
        .stored_ways = std.ArrayList(i64).init(alloc),
    };
    defer userdata.deinit();

    try runParser(args.osm_data, .{
        .ctx = &userdata,
        .startElement = startElement,
        .endElement = endElement,
    });
    var it = userdata.way_id_colour_map.keyIterator();
    while (it.next()) |way_key| {
        const channels = userdata.way_id_colour_map.get(way_key.*) orelse {
            const white: [4]f32 = .{ 0.0, 0.0, 0.0, 0.0 };
            _ = try userdata.points_out.writeAll(std.mem.asBytes(&white));
            continue;
        };
        userdata.log.print("Adding colour ({d}, {d}, {d}, {d}) to way {}\n", .{ channels[0] * 255, channels[1] * 255, channels[2] * 255, channels[3] * 255, way_key.* }) catch |err| {
            try userdata.log.print("Error printing: {}\n", .{err});
        };
        for (channels) |channel| {
            _ = try userdata.points_out.writeAll(std.mem.asBytes(&channel));
        }
    }
    const metadata_out_f = try std.fs.cwd().createFile(metadata_path, .{});
    userdata.metadata.end_nodes = userdata.num_nodes * 8;
    userdata.metadata.end_refs = userdata.metadata.end_nodes + (userdata.num_refs * 4);
    userdata.metadata.end_ways = userdata.metadata.end_refs + (userdata.num_ways * 4);

    try std.json.stringify(userdata.metadata, .{}, metadata_out_f.writer());
}
