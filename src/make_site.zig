const std = @import("std");
const builtin = @import("builtin");
const Metadata = @import("Metadata.zig");
const XmlParser = @import("XmlParser.zig");
const Allocator = std.mem.Allocator;

const node_obj = struct {
    idx: u64,
    lat: f32,
    lon: f32,
    colour: [4]f32 = .{ 1.0, 1.0, 1.0, 1.0 },
};

const way_obj = struct {
    nodes: std.ArrayList(u64),
    way_type: u32 = 0,
};

const Userdata = struct {
    allocator: *Allocator,
    points_out: std.io.AnyWriter,
    el_out: std.io.AnyWriter,
    log: std.io.AnyWriter,
    metadata: Metadata = .{},
    node_map: std.AutoArrayHashMap(u64, node_obj),
    way_map: std.AutoArrayHashMap(u64, []u64),
    way_list: std.ArrayList(u64),
    //node_id_idx_map: std.AutoHashMap(i64, usize),
    //node_id_coord_map: std.AutoHashMap(i64, [2]f32),
    num_nodes: u64 = 0,
    num_refs: u64 = 0,
    num_ways: u64 = 0,
    //way_id_colour_map: std.AutoHashMap(i64, [4]f32),
    //node_idx_list: std.ArrayList(usize),
    current_way: way_obj = undefined,
    colour_buf: [32]u8 = undefined,
    stored_colour: [4]f32 = .{ 1.0, 1.0, 1.0, 1.0 },

    fn deinit(self: *Userdata) void {
        //self.node_id_idx_map.deinit();
        //self.way_id_colour_map.deinit();
        self.node_map.clearAndFree();
        self.node_map.deinit();
        self.way_map.clearAndFree();
        self.way_map.deinit();
        self.way_list.clearAndFree();
        self.way_list.deinit();
        //self.node_id_coord_map.deinit();
        //self.stored_ways.deinit();
        //self.node_idx_list.deinit();
    }

    fn wayInit(self: *Userdata) void {
        self.current_way = way_obj{ .nodes = std.ArrayList(u64).init(self.allocator.*) };
    }

    fn wayDeinit(self: *Userdata) void {
        self.current_way.nodes.deinit();
        self.current_way = undefined;
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
        const node_id = std.fmt.parseInt(u64, node_id_s, 10) catch return;

        const node = node_obj{ .idx = user_data.num_nodes, .lat = lat, .lon = lon };

        //_ = std.fmt.bufPrint(node.colour, "{s}", .{"#FFFFFF"}) catch unreachable;
        user_data.node_map.put(node_id, node) catch unreachable;

        //user_data.node_id_idx_map.put(node_id, user_data.num_nodes) catch return;
        //user_data.node_id_coord_map.put(node_id, .{ lon, lat }) catch return;

        user_data.metadata.max_lon = @max(lon, user_data.metadata.max_lon);
        user_data.metadata.min_lon = @min(lon, user_data.metadata.min_lon);
        user_data.metadata.max_lat = @max(lat, user_data.metadata.max_lat);
        user_data.metadata.min_lat = @min(lat, user_data.metadata.min_lat);

        //std.debug.assert(builtin.cpu.arch.endian() == .little);
        //user_data.points_out.writeAll(std.mem.asBytes(&lon)) catch unreachable;
        //user_data.points_out.writeAll(std.mem.asBytes(&lat)) catch unreachable;
    }

    fn handleWay(user_data: *Userdata, _: *XmlParser.XmlAttrIter) void {
        //user_data.log.print("{any} \n", .{attrs[0],}) catch unreachable;
        //user_data.stored_colour = std.fmt.bufPrint(&user_data.colour_buf, "{s}", .{"#FFFFFF"}) catch unreachable;
        defer user_data.num_refs += 1;

        defer user_data.num_ways += 1;

        //user_data.points_out.writeInt(u32, 0xffffffff, .little) catch unreachable;
        user_data.wayInit();
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

fn getColour(colour_str: []u8, _: *Userdata) [4]f32 {
    var colour: [4]f32 = .{ 1.0, 1.0, 1.0, 1.0 };

    if (colour_str[0] == '#') {
        const r_channel: f32 = @as(f32, @floatFromInt(std.fmt.parseInt(i32, colour_str[1..3], 16) catch 255));
        const g_channel: f32 = @as(f32, @floatFromInt(std.fmt.parseInt(i32, colour_str[3..5], 16) catch 255));
        const b_channel: f32 = @as(f32, @floatFromInt(std.fmt.parseInt(i32, colour_str[5..7], 16) catch 255));
        const alpha_channel: f32 = 255;
        //if (colour_str.len == 9) {
        //    alpha_channel = @as(f32, @floatFromInt(std.fmt.parseInt(i32, colour_str[7..9], 16) catch 255));
        //}

        colour = .{
            r_channel / 255,
            g_channel / 255,
            b_channel / 255,
            alpha_channel / 255,
        };
    }

    if (std.mem.eql(u8, colour_str[0..2], "red")) {
        colour = .{ 1.0, 0.0, 0.0, 1.0 };
    } else if (std.mem.eql(u8, colour_str[0..4], "green")) {
        colour = .{ 0.0, 1.0, 0.0, 1.0 };
    } else if (std.mem.eql(u8, colour_str[0..3], "blue")) {
        colour = .{ 0.0, 0.0, 1.0, 1.0 };
    }

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

        const node_id = std.fmt.parseInt(u64, node_id_s, 10) catch unreachable;
        user_data.current_way.nodes.append(node_id) catch unreachable;
    } else if (std.mem.eql(u8, name, "tag")) {
        const tagName = findAttributeVal("k", attrs) orelse return;
        const tagVal = findAttributeVal("v", attrs) orelse return;

        if (((std.mem.eql(u8, tagName, "highway")) and (std.mem.eql(u8, tagVal, "footway") or std.mem.eql(u8, tagVal, "path") or std.mem.eql(u8, tagVal, "cycleway"))) or ((std.mem.eql(u8, tagName, "foot") or std.mem.eql(u8, tagName, "cycle"))) and (std.mem.eql(u8, tagVal, "yes") or std.mem.eql(u8, tagVal, "designated"))) {
            //user_data.current_way.colour = .{ 0.0, 1.0, 0.0, 1.0 };
            user_data.current_way.way_type = 1;
        }
        if (std.mem.eql(u8, tagName, "railway")) {
            //user_data.current_way.colour = .{ 0.0, 0.0, 1.0, 1.0 };
            user_data.current_way.way_type = 2;
        }
        if (std.mem.eql(u8, tagName, "building") and !std.mem.eql(u8, tagVal, "roof")) {
            //user_data.current_way.colour = .{ 0.0, 0.0, 1.0, 1.0 };
            user_data.current_way.way_type = 3;
        }
    }
}

fn endElement(ctx: ?*anyopaque, name: []const u8) void {
    const user_data: *Userdata = @ptrCast(@alignCast(ctx));
    if (std.mem.eql(u8, name, "way")) {
        //const way_list = user_data.way_list.toOwnedSlice() catch unreachable;

        //std.debug.print("Assigning: Way: {} has {} nodes: {any}...\n", .{ user_data.current_way, way_list.len, way_list[0..@min(5, way_list.len - 1)] });
        //defer user_data.way_list.clearAndFree();
        //defer user_data.way_list.allocator.free(way_list);
        //user_data.log.print("Putting Way: {} with {} nodes\n", .{ user_data.current_way, way_list_dupe.len }) catch unreachable;
        //const way = way_obj{ .nodes = way_list };
        user_data.el_out.writeInt(u32, 0xffffffff, .little) catch unreachable;
        user_data.el_out.writeInt(u32, user_data.current_way.way_type, .little) catch unreachable;
        user_data.log.print("Putting Way type {} with {} nodes\n", .{ user_data.current_way.way_type, user_data.current_way.nodes.items.len }) catch unreachable;
        for (user_data.current_way.nodes.items) |node_id| {
            const node = user_data.node_map.get(node_id) orelse unreachable;
            user_data.el_out.writeInt(u32, @intCast(node.idx), .little) catch unreachable;
        }

        user_data.wayDeinit();
    } else if (std.mem.eql(u8, name, "osm")) {
        var node_iter = user_data.node_map.iterator();
        while (node_iter.next()) |node_entry| {
            const node = node_entry.value_ptr.*;
            user_data.points_out.writeAll(std.mem.asBytes(&node.lon)) catch unreachable;
            user_data.points_out.writeAll(std.mem.asBytes(&node.lat)) catch unreachable;

            //for (node.colour) |channel| {
            //user_data.points_out.writeAll(std.mem.asBytes(&channel)) catch unreachable;
            //}
        }
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
    var gpa = std.heap.GeneralPurposeAllocator(.{ .verbose_log = true }){};
    var gpa2 = std.heap.GeneralPurposeAllocator(.{ .verbose_log = true }){};
    defer _ = gpa2.deinit();
    defer _ = gpa.deinit();

    const alloc = gpa.allocator();
    var alloc2 = gpa2.allocator();

    var args = try Args.parse(alloc);
    defer args.deinit();

    try std.fs.cwd().makePath(args.output);

    try linkFile(alloc, args.index_wasm, args.output);
    try linkWww(alloc, args.input_www, args.output);

    const map_data_path = try std.fs.path.join(alloc, &.{ args.output, "map_data.bin" });
    defer alloc.free(map_data_path);

    const element_data_path = try std.fs.path.join(alloc, &.{ args.output, "element_data.bin" });
    defer alloc.free(element_data_path);

    const metadata_path = try std.fs.path.join(alloc, &.{ args.output, "map_data.json" });
    defer alloc.free(metadata_path);

    const out_f = try std.fs.cwd().createFile(map_data_path, .{});
    var points_out_buf_writer = std.io.bufferedWriter(out_f.writer());
    defer points_out_buf_writer.flush() catch unreachable;
    const points_out_writer = points_out_buf_writer.writer().any();

    const el_out_f = try std.fs.cwd().createFile(element_data_path, .{});
    var el_out_buf_writer = std.io.bufferedWriter(el_out_f.writer());
    defer el_out_buf_writer.flush() catch unreachable;
    const el_out_writer = el_out_buf_writer.writer().any();

    const log_f = try std.fs.cwd().createFile("make_site.log", .{});
    var log_buf_writer = std.io.bufferedWriter(log_f.writer());
    defer log_buf_writer.flush() catch unreachable;
    const log_writer = log_buf_writer.writer().any();

    var userdata = Userdata{
        .allocator = &alloc2,
        .points_out = points_out_writer,
        .el_out = el_out_writer,
        .log = log_writer,
        .node_map = std.AutoArrayHashMap(u64, node_obj).init(alloc),
        .way_map = std.AutoArrayHashMap(u64, []u64).init(alloc),
        .way_list = std.ArrayList(u64).init(alloc),
        //.node_id_idx_map = std.AutoHashMap(i64, usize).init(alloc),
        //.way_id_colour_map = std.AutoHashMap(i64, [4]f32).init(alloc),
        //.stored_ways = std.ArrayList(i64).init(alloc),
    };
    defer userdata.deinit();

    try runParser(args.osm_data, .{
        .ctx = &userdata,
        .startElement = startElement,
        .endElement = endElement,
    });

    const metadata_out_f = try std.fs.cwd().createFile(metadata_path, .{});
    userdata.metadata.end_nodes = userdata.num_nodes * 8;
    userdata.metadata.end_refs = userdata.metadata.end_nodes + (userdata.num_refs * 4);
    userdata.metadata.end_ways = userdata.metadata.end_refs + (userdata.num_ways * 4);

    try std.json.stringify(userdata.metadata, .{}, metadata_out_f.writer());
}
