const std = @import("std");
const way_types = enum { Foot, Bicycle, Car, Building, Rail, None };

const Way = struct {
    nodes: []usize,
};

const Building = struct {
    nodes: []usize,
    height: f32,
};

const WayList = std.MultiArrayList(Way);
const BuildingList = std.MultiArrayList(Building);

fn WayListDeinit(waylist: *WayList, alloc: *std.mem.Allocator) void {
    var waylist_deref = waylist.*;
    while (waylist_deref.popOrNull()) |item| {
        alloc.free(item.nodes);
    }
    waylist_deref.deinit(alloc.*);
}

fn BuildingListDeinit(waylist: *BuildingList, alloc: *std.mem.Allocator) void {
    var waylist_deref = waylist.*;
    while (waylist_deref.popOrNull()) |item| {
        alloc.free(item.nodes);
        alloc.destroy(&item.height);
    }
    waylist_deref.deinit(alloc.*);
}

const WayCollection = struct {
    foot: WayList,
    bicycle: WayList,
    car: WayList,
    building: BuildingList,
    rail: WayList,
    other: WayList,

    fn deinit(self: *WayCollection, alloc: *std.mem.Allocator) void {
        WayListDeinit(&self.foot, alloc);
        WayListDeinit(&self.bicycle, alloc);
        WayListDeinit(&self.car, alloc);
        BuildingListDeinit(&self.building, alloc);
        WayListDeinit(&self.rail, alloc);
        WayListDeinit(&self.other, alloc);
    }
};

const type_ref = struct {
    key: []const u8,
    values: [][]const u8 = undefined,
    not_values: [][]const u8 = undefined,
    type: way_types,
};

var foot_vals = [_][]const u8{ "path", "cycle", "footway", "pedestrian", "steps" };
const foot = type_ref{
    .key = "highway",
    .values = &foot_vals,
    .type = way_types.Foot,
};

const rail = type_ref{
    .key = "railway",
    .type = way_types.Rail,
};

var building_not_vals = [_][]const u8{
    "roof",
};
const building = type_ref{
    .key = "building",
    .not_values = &building_not_vals,
    .type = way_types.Building,
};

const types: [3]type_ref = .{ foot, rail, building };

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

    fn parse(alloc: std.mem.Allocator) !Args {
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

const metadata_t = struct {
    min_lat: f32,
    max_lat: f32,
    min_lon: f32,
    max_lon: f32,
    n_nodes: usize,
    n_buildings: usize,
    n_foot: usize,
    n_bicycle: usize,
    n_car: usize,
    n_rail: usize,
    n_other: usize,
};

fn addWay(ctx: *metadata_t, wayType: way_types) void {
    switch (wayType) {
        way_types.Foot => ctx.n_foot += 1,
        way_types.Bicycle => ctx.n_bicycle += 1,
        way_types.Car => ctx.n_car += 1,
        way_types.Building => ctx.n_buildings += 1,
        way_types.Rail => ctx.n_rail += 1,
        else => ctx.n_other += 1,
    }
}

const modes = enum { Node, Way, Relation, None };

pub fn parseChunk(ctx: *metadata_t, chunk: []const u8, node_list: *std.AutoArrayHashMap(usize, [2]f32), node_refs: *std.AutoArrayHashMap(usize, usize), way_collection: *WayCollection, alloc: *std.mem.Allocator) !void {
    var position: u32 = 0;
    var mode = modes.None;

    while (true) {
        defer position += 1;
        if (position >= chunk.len) {
            break;
        }
        //std.debug.print("{d}:{c} mode: {s}               \n", .{ position, chunk[position], @tagName(mode) });
        switch (mode) {
            modes.None => {
                if (chunk[position] == '<') {
                    if (chunk[position + 1] == 'n' and chunk[position + 2] == 'o' and chunk[position + 3] == 'd' and chunk[position + 4] == 'e') {
                        mode = modes.Node;
                    } else if (chunk[position + 1] == 'w' and chunk[position + 2] == 'a' and chunk[position + 3] == 'y') {
                        mode = modes.Way;
                    }
                }
            },
            modes.Node => {
                var id: usize = 0;
                var lat: f32 = 0.0;
                var lon: f32 = 0.0;
                while (true) {
                    if (chunk[position] == ' ' and chunk[position + 1] == 'i' and chunk[position + 2] == 'd') {
                        position += 5;
                        const start = position;
                        var end = position;
                        while (chunk[end] != '"') {
                            end += 1;
                        }
                        id = std.fmt.parseInt(usize, chunk[start..end], 10) catch unreachable;
                    } else if (chunk[position] == 'l' and chunk[position + 1] == 'a' and chunk[position + 2] == 't') {
                        position += 5;
                        const start = position;
                        var end = position;
                        while (chunk[end] != '"') {
                            end += 1;
                        }

                        lat = std.fmt.parseFloat(f32, chunk[start..end]) catch unreachable;
                    } else if (chunk[position] == 'l' and chunk[position + 1] == 'o' and chunk[position + 2] == 'n') {
                        position += 5;
                        const start = position;
                        var end = position;
                        while (chunk[end] != '"') {
                            end += 1;
                        }
                        lon = std.fmt.parseFloat(f32, chunk[start..end]) catch unreachable;
                    } else if (chunk[position] == '>') {
                        break;
                    }
                    position += 1;
                }
                //std.debug.print("Node: id={d}, lat={d}, lon={d}\n", .{ id, lat, lon });
                if (lat < ctx.min_lat) {
                    ctx.min_lat = lat;
                }
                if (lat > ctx.max_lat) {
                    ctx.max_lat = lat;
                }
                if (lon < ctx.min_lon) {
                    ctx.min_lon = lon;
                }
                if (lon > ctx.max_lon) {
                    ctx.max_lon = lon;
                }
                ctx.n_nodes += 1;
                node_list.put(id, .{ lat, lon }) catch unreachable;
                node_refs.put(id, node_list.count() - 1) catch unreachable;
                mode = modes.None;
            },
            modes.Way => {
                var id: usize = 0;
                var way_type = way_types.None;
                var nodes = std.ArrayList(usize).init(alloc.*);

                var building_height: f32 = 0.0;
                var key_list = std.ArrayList([]const u8).init(alloc.*);
                var val_list = std.ArrayList([]const u8).init(alloc.*);

                defer {
                    nodes.deinit();
                    key_list.deinit();
                    val_list.deinit();
                }
                while (true) {
                    //std.debug.print("\r{c}", .{chunk[position]});
                    if (chunk[position] == ' ' and chunk[position + 1] == 'i' and chunk[position + 2] == 'd') {
                        position += 5;
                        const start = position;

                        while (chunk[position] != '"') {
                            position += 1;
                        }
                        const end = position;
                        id = std.fmt.parseInt(usize, chunk[start..end], 10) catch unreachable;
                        position += 1;
                    } else if (chunk[position] == ' ' and chunk[position + 1] == 'r' and chunk[position + 2] == 'e' and chunk[position + 3] == 'f' and chunk[position + 4] == '=') {
                        position += 6;
                        const start = position;

                        while (chunk[position] != '"') {
                            position += 1;
                        }
                        const end = position;
                        const node_id = std.fmt.parseInt(usize, chunk[start..end], 10) catch {
                            std.debug.print("Error parsing Way node ref: '{s}'\n", .{chunk[start..end]});
                            unreachable;
                        };

                        const node_idx = node_refs.get(node_id) orelse unreachable;

                        nodes.append(node_idx) catch unreachable;
                        position += 1;
                    } else if (chunk[position] == '<' and chunk[position + 1] == 't' and chunk[position + 2] == 'a' and chunk[position + 3] == 'g') {
                        while (chunk[position] != '>') {
                            if (chunk[position] == ' ' and chunk[position + 1] == 'k' and chunk[position + 2] == '=' and chunk[position + 3] == '"') {
                                position += 4;
                                const start = position;

                                while (chunk[position] != '"') {
                                    position += 1;
                                }
                                const end = position;

                                key_list.append(chunk[start..end]) catch unreachable;
                            } else if (chunk[position] == ' ' and chunk[position + 1] == 'v' and chunk[position + 2] == '=' and chunk[position + 3] == '"') {
                                position += 4;
                                const start = position;
                                while (chunk[position] != '"') {
                                    position += 1;
                                }
                                const end = position;

                                val_list.append(chunk[start..end]) catch unreachable;
                            }
                            position += 1;
                        }
                    } else if (chunk[position] == '/' and chunk[position + 1] == 'w') {
                        break;
                    } else {
                        position += 1;
                    }

                    for (0.., key_list.items) |idx, key| {

                        //std.debug.print("   Key: {s}, Value: {s}\n", .{ key_buff[0..key_len], val_buff[0..val_len] });
                        for (types) |wtype| {
                            var key_match = false;
                            var val_match = false;
                            var not_val_match = false;
                            if (std.mem.eql(u8, key, wtype.key)) {
                                key_match = true;
                                if (wtype.values.len > 0) {
                                    for (wtype.values) |value| {
                                        _ = std.mem.indexOf(u8, val_list.items[idx], value) orelse continue;
                                        val_match = true;
                                        //std.debug.print("   Comparing: {s} == {s} -> {any}\n", .{ val_buff[0..val_len], value, val_match });
                                        break;
                                    }
                                } else {
                                    val_match = true;
                                }
                                if (wtype.not_values.len > 0) {
                                    for (wtype.not_values) |value| {
                                        _ = std.mem.indexOf(u8, val_list.items[idx], value) orelse {
                                            not_val_match = true;
                                            break;
                                        };
                                        not_val_match = false;
                                    }
                                } else {
                                    not_val_match = true;
                                }
                            }
                            if (key_match and val_match and not_val_match) {
                                //std.debug.print("Way type: {s}\n", .{@tagName(wtype.type)});
                                way_type = wtype.type;
                                break;
                            }
                        }

                        if (std.mem.eql(u8, key, "building:levels") and building_height == 0.0) {
                            building_height = std.fmt.parseFloat(f32, val_list.items[idx]) catch unreachable;
                            building_height *= 2.7;
                        } else if (std.mem.eql(u8, key, "height")) {
                            building_height = std.fmt.parseFloat(f32, val_list.items[idx]) catch unreachable;
                        }
                    }
                }

                const node_array = nodes.toOwnedSlice() catch unreachable;
                addWay(ctx, way_type);
                switch (way_type) {
                    way_types.Foot => way_collection.foot.append(alloc.*, .{ .nodes = node_array }) catch unreachable,
                    way_types.Bicycle => way_collection.bicycle.append(alloc.*, .{ .nodes = node_array }) catch unreachable,
                    way_types.Car => way_collection.car.append(alloc.*, .{ .nodes = node_array }) catch unreachable,
                    way_types.Rail => way_collection.rail.append(alloc.*, .{ .nodes = node_array }) catch unreachable,
                    way_types.Building => way_collection.building.append(alloc.*, .{ .nodes = node_array, .height = building_height }) catch unreachable,
                    else => way_collection.other.append(alloc.*, .{ .nodes = node_array }) catch unreachable,
                }

                mode = modes.None;
            },
            else => {
                continue;
            },
        }
    }
}

pub fn parseOSM(ctx: *metadata_t, data_path: []const u8, node_list: *std.AutoArrayHashMap(usize, [2]f32), node_refs: *std.AutoArrayHashMap(usize, usize), way_list: *WayCollection, alloc: *std.mem.Allocator) !void {
    const f = try std.fs.cwd().openFile(data_path, .{});
    defer f.close();

    std.debug.print("Parsing map\n", .{});
    const mb = 1024 * 1024;
    const file_content = f.readToEndAlloc(alloc.*, 4 * mb) catch unreachable;
    defer alloc.free(file_content);

    try parseChunk(ctx, file_content, node_list, node_refs, way_list, alloc);

    std.debug.print("done\n", .{});
    std.debug.print("node_list items: {d}\n", .{node_list.count()});
    std.debug.print("foot items: {d}\n", .{way_list.foot.len});
    std.debug.print("bicycle items: {d}\n", .{way_list.bicycle.len});
    std.debug.print("car items: {d}\n", .{way_list.car.len});
    std.debug.print("building items: {d}\n", .{way_list.building.len});
    std.debug.print("rail items: {d}\n", .{way_list.rail.len});
    std.debug.print("other items: {d}\n", .{way_list.other.len});
}

fn writeWays(writer: *std.io.AnyWriter, way_list: *WayList, alloc: *std.mem.Allocator) void {
    while (way_list.popOrNull()) |way| {
        writer.writeInt(u32, 0xffffffff, .little) catch unreachable;
        for (way.nodes) |node| {
            writer.writeInt(u32, @intCast(node), .little) catch unreachable;
        }

        alloc.free(way.nodes);
    }
}

fn writeBuildings(writer: *std.io.AnyWriter, building_list: *BuildingList, alloc: *std.mem.Allocator) void {
    while (building_list.popOrNull()) |way| {
        writer.writeInt(u32, 0xffffffff, .little) catch unreachable;
        for (way.nodes) |node| {
            writer.writeInt(u32, @intCast(node), .little) catch unreachable;
        }
        alloc.free(way.nodes);
    }
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();

    var alloc = gpa.allocator();

    //var way_list = std.MultiArrayList(Way).init(alloc);

    var args = try Args.parse(alloc);
    defer args.deinit();

    try std.fs.cwd().makePath(args.output);

    const map_data_path = try std.fs.path.join(alloc, &.{ args.output, "map_data.bin" });
    defer alloc.free(map_data_path);

    const element_data_path = try std.fs.path.join(alloc, &.{ args.output, "element_data.bin" });
    defer alloc.free(element_data_path);

    const metadata_path = try std.fs.path.join(alloc, &.{ args.output, "map_data.json" });
    defer alloc.free(metadata_path);

    var node_list = std.AutoArrayHashMap(usize, [2]f32).init(alloc);
    defer {
        node_list.deinit();
    }

    var node_refs = std.AutoArrayHashMap(usize, usize).init(alloc);
    defer node_refs.deinit();

    var way_collection = WayCollection{
        .foot = WayList{},
        .bicycle = WayList{},
        .car = WayList{},
        .building = BuildingList{},
        .rail = WayList{},
        .other = WayList{},
    };
    defer {
        way_collection.deinit(&alloc);
    }

    var metadata = metadata_t{
        .min_lat = 90,
        .max_lat = -90,
        .min_lon = 180,
        .max_lon = -180,
        .n_nodes = 0,
        .n_buildings = 0,
        .n_foot = 0,
        .n_bicycle = 0,
        .n_car = 0,
        .n_rail = 0,
        .n_other = 0,
    };

    try parseOSM(&metadata, args.osm_data, &node_list, &node_refs, &way_collection, &alloc);

    const metadata_out_f = try std.fs.cwd().createFile(metadata_path, .{});
    defer metadata_out_f.close();

    const data_out_f = std.fs.cwd().createFile(map_data_path, .{}) catch unreachable;
    defer data_out_f.close();

    var data_buffered_writer = std.io.bufferedWriter(data_out_f.writer());
    defer data_buffered_writer.flush() catch unreachable;

    var data_writer = data_out_f.writer().any();

    const nodes = node_list.values();

    for (nodes) |node| {
        data_writer.writeAll(std.mem.asBytes(&node[0])) catch unreachable;
        data_writer.writeAll(std.mem.asBytes(&node[1])) catch unreachable;
    }

    std.debug.print("Way 1: {any}\n", .{way_collection.foot.get(way_collection.foot.len - 1).nodes});
    std.debug.print("Last other: {any}\n", .{way_collection.other.get(0).nodes});

    writeWays(&data_writer, &way_collection.foot, &alloc);
    writeWays(&data_writer, &way_collection.bicycle, &alloc);
    writeWays(&data_writer, &way_collection.car, &alloc);
    writeBuildings(&data_writer, &way_collection.building, &alloc);
    writeWays(&data_writer, &way_collection.rail, &alloc);
    writeWays(&data_writer, &way_collection.other, &alloc);

    std.json.stringify(
        metadata,
        .{},
        metadata_out_f.writer(),
    ) catch unreachable;
}
