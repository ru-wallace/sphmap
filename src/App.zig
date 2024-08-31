const std = @import("std");
const Metadata = @import("Metadata.zig");

const App = @This();

mouse_tracker: MouseTracker = .{},
renderer: Renderer,

pub fn init(aspect_val: f32, map_data: []const u8, metadata: Metadata) App {
    var ret = .{ .renderer = Renderer.init(aspect_val, map_data, metadata) };

    ret.renderer.render();

    return ret;
}

pub fn onMouseDown(self: *App, x: f32, y: f32) void {
    self.mouse_tracker.onDown(x, y);
}

pub fn onMouseUp(self: *App) void {
    self.mouse_tracker.onUp();
}

pub fn onMouseMove(self: *App, x: f32, y: f32) void {
    if (self.mouse_tracker.down) {
        const movement = self.mouse_tracker.getMovement(x, y);
        self.mouse_tracker.onDown(x, y);
        self.renderer.lon_center.val -= movement.x / self.renderer.zoom.val * 2;
        self.renderer.lat_center.val += movement.y / self.renderer.zoom.val * 2;
        self.renderer.render();
    }
}

pub fn zoomIn(self: *App) void {
    self.renderer.zoom.val *= 2.0;
    self.renderer.render();
}

pub fn zoomOut(self: *App) void {
    self.renderer.zoom.val *= 0.5;
    self.renderer.render();
}

const NormalizedPosition = struct {
    x: f32,
    y: f32,
};

const NormalizedOffset = struct {
    x: f32,
    y: f32,
};

const MouseTracker = struct {
    down: bool = false,
    pos: NormalizedPosition = undefined,

    pub fn onDown(self: *MouseTracker, x: f32, y: f32) void {
        self.down = true;
        self.pos.x = x;
        self.pos.y = y;
    }

    pub fn onUp(self: *MouseTracker) void {
        self.down = false;
    }

    pub fn getMovement(self: *MouseTracker, x: f32, y: f32) NormalizedOffset {
        return .{
            .x = x - self.pos.x,
            .y = y - self.pos.y,
        };
    }
};

const FloatUniform = struct {
    loc: i32,
    val: f32 = 0.0,

    fn init(program: i32, key: []const u8, val: f32) FloatUniform {
        const loc = js.glGetUniformLoc(program, key.ptr, key.len);
        return .{
            .loc = loc,
            .val = val,
        };
    }
    fn set(self: *FloatUniform) void {
        js.glUniform1f(self.loc, self.val);
    }
};

const VecUniform = struct {
    loc: i32,
    val: [4]f32 = undefined,

    fn init(program: i32, key: []const u8, val: [4]f32) VecUniform {
        const loc = js.glGetUniformLoc(program, key.ptr, key.len);
        return .{
            .loc = loc,
            .val = val,
        };
    }

    fn set(self: *VecUniform) void {
        js.glUniform4f(self.loc, self.val[0], self.val[1], self.val[2], self.val[3]);
    }
};

const IUniform = struct {
    loc: i32,
    val: u32,

    fn init(program: i32, key: []const u8, val: u32) IUniform {
        const loc = js.glGetUniformLoc(program, key.ptr, key.len);
        return .{
            .loc = loc,
            .val = val,
        };
    }

    fn set(self: *IUniform) void {
        js.glUniform1i(self.loc, self.val);
    }
};

fn setUniforms(uniforms: []const FloatUniform) void {
    for (uniforms) |uniform| {
        js.glUniform1f(uniform.loc, uniform.val);
    }
}

const js = struct {
    extern fn logWasm(s: [*]const u8, len: usize) void;
    extern fn compileLinkProgram(vs: [*]const u8, vs_len: usize, fs: [*]const u8, fs_len: usize) i32;
    extern fn bind2DFloat32Data(data: [*]const f32, data_len: usize) i32;
    extern fn bindEbo(data: [*]const u32, data_len: usize) i32;
    extern fn glBindVertexArray(vao: i32) void;
    extern fn glClearColor(r: f32, g: f32, b: f32, a: f32) void;
    extern fn glClear(mask: i32) void;
    extern fn glUseProgram(program: i32) void;
    extern fn glDrawArrays(mode: i32, first: i32, last: i32) void;
    extern fn glDrawElements(mode: i32, count: i32, type: i32, offs: i32) void;
    extern fn glGetUniformLoc(program: i32, name: [*]const u8, name_len: usize) i32;
    extern fn glUniform1f(loc: i32, val: f32) void;
    extern fn glUniform4f(loc: i32, val1: f32, val2: f32, val3: f32, val4: f32) void;
    extern fn glUniform1i(loc: i32, val: u32) void;
    extern fn zigPrint(s: [*]const u8, len: usize) void;
};

fn print(comptime fmt: []const u8, args: anytype) void {
    var buf: [4096]u8 = undefined;
    const slice = std.fmt.bufPrint(&buf, fmt, args) catch unreachable;
    js.logWasm(slice.ptr, slice.len);
}

const Gl = struct {
    // https://registry.khronos.org/webgl/specs/latest/1.0/
    const COLOR_BUFFER_BIT = 0x00004000;
    const POINTS = 0x0000;
    const LINE_STRIP = 0x0003;
    const UNSIGNED_INT = 0x1405;
};

const vs_source = @embedFile("vertex.vert");
const rs_source = @embedFile("roof.vert");
const fs_source = @embedFile("fragment.frag");

const element_types = struct {
    other: []u32,
    rail: []u32,
    foot: []u32,
    building: []u32,

    fn addWay(al: *std.ArrayList(u32), index_data: []const u32, i: usize) usize {
        //var array_list = al.*;
        al.append(0xFFFFFFFF) catch unreachable;
        var idx = i + 1;
        var count: usize = 0;
        while (idx < index_data.len and index_data[idx] != 0xFFFFFFFF) {
            if (idx < 100) {
                print("idx:{} Count: {} Adding data 0x{x} array_size: {}", .{ idx, count, index_data[idx], al.items.len });
            }

            al.append(index_data[idx]) catch unreachable;
            idx += 1;
            count += 1;
        }
        return idx;
    }

    pub fn processData(data: []const u32) element_types {
        var rail = std.ArrayList(u32).init(std.heap.wasm_allocator);
        var foot = std.ArrayList(u32).init(std.heap.wasm_allocator);
        var building = std.ArrayList(u32).init(std.heap.wasm_allocator);
        var other = std.ArrayList(u32).init(std.heap.wasm_allocator);
        defer {
            rail.deinit();
            foot.deinit();
            building.deinit();
            other.deinit();
        }

        var i: usize = 0;
        while (i < data.len) {
            defer i += 1;
            const val = data[i];
            print("i{} Val: {}", .{ i, val });
            if (val == 1) {
                print("Adding Foot", .{});
                i = addWay(&foot, data, i);
            } else if (val == 2) {
                print("Adding Rail", .{});
                i = addWay(&rail, data, i);
            } else if (val == 3) {
                print("Adding Building", .{});
                i = addWay(&building, data, i);
            } else {
                print("Adding Other", .{});
                i = addWay(&other, data, i);
            }
        }
        const rail_arr: []u32 = std.heap.wasm_allocator.alloc(u32, rail.items.len) catch unreachable;
        const foot_arr: []u32 = std.heap.wasm_allocator.alloc(u32, foot.items.len) catch unreachable;
        const building_arr: []u32 = std.heap.wasm_allocator.alloc(u32, building.items.len) catch unreachable;
        const other_arr: []u32 = std.heap.wasm_allocator.alloc(u32, other.items.len) catch unreachable;

        @memcpy(rail_arr, rail.items);
        @memcpy(foot_arr, foot.items);
        @memcpy(building_arr, building.items);
        @memcpy(other_arr, other.items);
        print("Sizes: r{}, f{}, b{} o{}", .{ rail_arr.len, foot_arr.len, building_arr.len, other_arr.len });
        return .{
            .rail = rail_arr,
            .foot = foot_arr,
            .building = building_arr,
            .other = other_arr,
        };
    }
};

const Renderer = struct {
    program: i32,
    vao: i32,
    lat_center: FloatUniform,
    lon_center: FloatUniform,
    aspect: FloatUniform,
    zoom: FloatUniform,
    colour: VecUniform,
    height: FloatUniform,
    num_line_segments: usize,
    other: []u32,
    rail: []u32,
    foot: []u32,
    building: []u32,

    pub fn init(aspect_val: f32, map_data: []const u8, metadata: Metadata) Renderer {
        // Now create an array of positions for the square.
        const program = js.compileLinkProgram(vs_source, vs_source.len, fs_source, fs_source.len);
        //const roof_program = js.compileLinkProgram(rs_source, rs_source.len, fs_source, fs_source.len);

        const point_data: []const f32 = @alignCast(std.mem.bytesAsSlice(f32, map_data[0..@intCast(metadata.end_nodes)]));
        const vao = js.bind2DFloat32Data(point_data.ptr, point_data.len);

        const lat_center = FloatUniform.init(program, "lat_center", (metadata.max_lat + metadata.min_lat) / 2.0);
        //const lat_center_roof = FloatUniform.init(roof_program, "lat_center", (metadata.max_lat + metadata.min_lat) / 2.0);
        const lon_center = FloatUniform.init(program, "lon_center", (metadata.max_lon + metadata.min_lon) / 2.0);
        //const lon_center_roof = FloatUniform.init(roof_program, "lon_center", (metadata.max_lon + metadata.min_lon) / 2.0);
        const zoom = FloatUniform.init(program, "zoom", 2.0 / (metadata.max_lon - metadata.min_lon));
        //const zoom_roof = FloatUniform.init(roof_program, "zoom", 2.0 / (metadata.max_lon - metadata.min_lon));

        const aspect = FloatUniform.init(program, "aspect", aspect_val);
        //const aspect_roof = FloatUniform.init(roof_program, "aspect", aspect_val);

        const colour = VecUniform.init(program, "vColor", .{ 1.0, 1.0, 1.0, 1.0 });

        const height = FloatUniform.init(program, "height", 0.0);

        const index_data: []const u32 = @alignCast(std.mem.bytesAsSlice(u32, map_data[@intCast(metadata.end_nodes)..]));
        print("Index start: {x}", .{index_data[0..40]});
        const element_data = element_types.processData(index_data);

        return .{
            .program = program,
            //.roof_program = roof_program,
            .vao = vao,
            .lat_center = lat_center,
            .lon_center = lon_center,
            .aspect = aspect,
            .zoom = zoom,
            .colour = colour,
            .height = height,
            .num_line_segments = index_data.len,

            .other = element_data.other,
            .rail = element_data.rail,
            .foot = element_data.foot,
            .building = element_data.building,
        };
    }

    pub fn setAspect(self: *Renderer, aspect: f32) void {
        self.aspect.val = aspect;
        self.render();
    }

    fn renderLayer(self: *Renderer, index_data: []u32, colour: [4]f32, update_elements: bool) void {
        //print("Binding foot", .{});
        if (update_elements) {
            _ = js.bindEbo(index_data.ptr, index_data.len);
        }

        self.colour.val = colour;
        self.colour.set();
        js.glDrawElements(Gl.LINE_STRIP, @intCast(index_data.len), Gl.UNSIGNED_INT, 0);
    }

    pub fn render(self: *Renderer) void {
        js.glBindVertexArray(self.vao);
        js.glClearColor(0.0, 0.0, 0.0, 1.0);
        js.glClear(Gl.COLOR_BUFFER_BIT);
        js.glUseProgram(self.program);
        //js.glUseProgram(self.program);
        //var isRoof = IUniform.init(self.program, "isBuildingRoof", 0);
        //isRoof.set();
        setUniforms(&.{ self.lat_center, self.lon_center, self.zoom, self.aspect });
        self.height.val = 0.0;
        self.height.set();
        self.renderLayer(self.other, .{ 1.0, 1.0, 1.0, 1.0 }, true);
        self.renderLayer(self.rail, .{ 0.0, 0.0, 1.0, 1.0 }, true);
        self.renderLayer(self.foot, .{ 0.0, 1.0, 0.0, 1.0 }, true);
        self.renderLayer(self.building, .{ 1.0, 0.0, 0.0, 1.0 }, true);

        self.height.val = 0.00002;
        self.height.set();
        self.renderLayer(self.building, .{ 1.0, 0.0, 1.0, 0.75 }, false);
        self.height.val = 0.00004;
        self.height.set();
        self.renderLayer(self.building, .{ 1.0, 0.0, 1.0, 0.75 }, false);
        self.height.val = 0.00006;
        self.height.set();
        self.renderLayer(self.building, .{ 1.0, 0.0, 0.0, 1.0 }, false);
    }
};
