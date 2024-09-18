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
        self.renderer.tX -= movement.x / self.renderer.zoom.val; // * 2;
        self.renderer.tY += movement.y / self.renderer.zoom.val; // * 2;
        print("Lat center: {}", .{self.renderer.lat_center.val + self.renderer.tY});
        print("Lon center: {}", .{self.renderer.lon_center.val + self.renderer.tX});
        js.reportTranslation(self.renderer.tX, self.renderer.tY, self.renderer.zPos);
        self.renderer.render();
    }
}

pub fn zoomIn(self: *App) void {
    self.renderer.viewAngle *= 0.5;
    if (self.renderer.viewAngle < 1.0) {
        self.renderer.viewAngle = 1.0;
    }
    js.reportFOV(self.renderer.viewAngle);

    print("fov: {d}", .{self.renderer.viewAngle});
    self.renderer.render();
}

pub fn zoomOut(self: *App) void {
    self.renderer.viewAngle *= 2.0;
    if (self.renderer.viewAngle > 180.0) {
        self.renderer.viewAngle = 180.0;
    }
    js.reportFOV(self.renderer.viewAngle);
    print("fov: {d}", .{self.renderer.viewAngle});
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

const Mat4Uniform = struct {
    loc: i32,
    val: [16]f32 = undefined,

    fn init(program: i32, key: []const u8, val: [16]f32) Mat4Uniform {
        const loc = js.glGetUniformLoc(program, key.ptr, key.len);
        return .{
            .loc = loc,
            .val = val,
        };
    }

    fn set(self: *Mat4Uniform) void {
        const matSlice = self.val[0..16];
        //print("Setting matrix: {any} Ptr {any}", .{ matSlice, matSlice.ptr });

        js.glUniformMatrix4fv(self.loc, false, matSlice.ptr);
    }
};

fn setUniforms(uniforms: []const FloatUniform) void {
    for (uniforms) |uniform| {
        js.glUniform1f(uniform.loc, uniform.val);
    }
}

/// Mat4 is a 4x4 matrix. The values are stored in row-major order (Note that this is the opposite of OpenGL's column-major order).
/// for certainty, use getRowMajor() to get the values in row-major order, and getColMajor() to get the values in column-major order.
const Mat4 = struct {
    values: [16]f32,
    /// Create a new 4x4 matrix from the given values. ENTER THE VALUES IN MATHEMATICAL MATRIX ORDER (ROW MAJOR)
    fn initRowMajor(values: [16]f32) Mat4 {
        return .{
            .values = values,
        };
    }
    /// Create a new 4x4 matrix from the given values. ENTER THE VALUES IN WEBGL MATRIX ORDER (COLUMN MAJOR)
    fn initColMajor(values: [16]f32) Mat4 {
        const mat = .{
            values[0], values[4], values[8],  values[12],
            values[1], values[5], values[9],  values[13],
            values[2], values[6], values[10], values[14],
            values[3], values[7], values[11], values[15],
        };
        return .{
            .values = mat,
        };
    }

    fn transpose(values: [16]f32) [16]f32 {
        return .{
            values[0], values[4], values[8],  values[12],
            values[1], values[5], values[9],  values[13],
            values[2], values[6], values[10], values[14],
            values[3], values[7], values[11], values[15],
        };
    }
    /// Get the values of the matrix in row-major order (For Maths purposes)
    fn getRowMajor(self: Mat4) [16]f32 {
        return self.values;
    }
    /// Get the values of the matrix in column-major order (For WebGL purposes)
    fn getColMajor(self: Mat4) [16]f32 {
        return Mat4.transpose(self.values);
    }

    fn webGLOrder(self: Mat4) [16]f32 {
        return self.getColMajor();
    }

    fn mathsOrder(self: Mat4) [16]f32 {
        return self.getRowMajor();
    }

    fn multiply(a: Mat4, b: Mat4) Mat4 {

        //get the values of the matrices in row-major order (normal maths order)
        const matA = a.values;
        const matB = b.values;

        //the number of values in each row of the matrix
        const row_mult: usize = 4;

        var matC: [16]f32 = undefined;

        for (0..4) |i| { //for each row of matrix A
            for (0..4) |j| { // take each column of matrix B
                var sum: f32 = 0.0;
                for (0..4) |k| { //and sum the products of the row of A and column of B
                    sum += matA[i * row_mult + k] * matB[k * row_mult + j]; //for matA, the row is i, and the column is k. For matB, the row is k, and the column is j
                }
                matC[i * row_mult + j] = sum; // row is i, column is j
            }
        }

        return Mat4.initRowMajor(matC);
    }

    fn printRowMajor(self: Mat4) void {
        print(" {d} {d} {d} {d}", .{ self.values[0], self.values[1], self.values[2], self.values[3] });
        print(" {d} {d} {d} {d}", .{ self.values[4], self.values[5], self.values[6], self.values[7] });
        print(" {d} {d} {d} {d}", .{ self.values[8], self.values[9], self.values[10], self.values[11] });
        print(" {d} {d} {d} {d}", .{ self.values[12], self.values[13], self.values[14], self.values[15] });
    }

    fn printColMajor(self: Mat4) void {
        print(" {d} {d} {d} {d}", .{ self.values[0], self.values[4], self.values[8], self.values[12] });
        print(" {d} {d} {d} {d}", .{ self.values[1], self.values[5], self.values[9], self.values[13] });
        print(" {d} {d} {d} {d}", .{ self.values[2], self.values[6], self.values[10], self.values[14] });
        print(" {d} {d} {d} {d}", .{ self.values[3], self.values[7], self.values[11], self.values[15] });
    }
};

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
    extern fn glUniformMatrix4fv(loc: i32, transpose: bool, ptr: [*]const f32) void;
    extern fn zigPrint(s: [*]const u8, len: usize) void;
    extern fn reportFOV(fov: f32) void;
    extern fn reportTranslation(x: f32, y: f32, z: f32) void;
    extern fn reportRotation(x: f32, y: f32, z: f32) void;
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
//const rs_source = @embedFile("roof.vert");
const fs_source = @embedFile("fragment.frag");

const element_types = struct {
    foot: []u32,
    bicycle: []u32,
    car: []u32,
    rail: []u32,
    other: []u32,
    building: []u32,

    fn addWay(al: *std.ArrayList(u32), index_data: []const u32, i: usize) usize {
        //var array_list = al.*;
        al.append(0xFFFFFFFF) catch unreachable;
        var idx = i;
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

    fn addBuilding(al: *std.ArrayList(u32), index_data: []const u32, i: usize) usize {
        //var array_list = al.*;
        al.append(0xFFFFFFFF) catch unreachable;
        var idx = i;
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

    pub fn processData(data: []const u32, metadata: *const Metadata) element_types {
        var foot = std.ArrayList(u32).init(std.heap.wasm_allocator);
        var bicycle = std.ArrayList(u32).init(std.heap.wasm_allocator);
        var car = std.ArrayList(u32).init(std.heap.wasm_allocator);
        var rail = std.ArrayList(u32).init(std.heap.wasm_allocator);
        var other = std.ArrayList(u32).init(std.heap.wasm_allocator);
        var building = std.ArrayList(u32).init(std.heap.wasm_allocator);

        defer {
            rail.deinit();
            foot.deinit();
            building.deinit();
            other.deinit();
        }

        var i: usize = 1;
        var nFoot: u64 = 0;
        var nBicycle: u64 = 0;
        var nCar: u64 = 0;
        var nRail: u64 = 0;
        var nOther: u64 = 0;
        var nBuilding: u64 = 0;
        while (i < data.len) {
            defer {
                i += 1;
            }

            if (nFoot < metadata.n_foot) {
                i = addWay(&foot, data, i);
                nFoot += 1;
            } else if (nBicycle < metadata.n_bicycle) {
                i = addWay(&bicycle, data, i);
                nBicycle += 1;
            } else if (nCar < metadata.n_car) {
                i = addWay(&car, data, i);
                nCar += 1;
            } else if (nRail < metadata.n_rail) {
                i = addWay(&rail, data, i);
                nRail += 1;
            } else if (nOther < metadata.n_other) {
                i = addWay(&other, data, i);
                nOther += 1;
            } else {
                i = addWay(&building, data, i);
                nBuilding += 1;
            }

            //print("Foot: {} Bicycle: {} Car: {} Rail: {} Other: {} Building: {}", .{ nFoot, nBicycle, nCar, nRail, nOther, nBuilding });
        }
        const foot_arr: []u32 = std.heap.wasm_allocator.alloc(u32, foot.items.len) catch unreachable;
        const bicycle_arr: []u32 = std.heap.wasm_allocator.alloc(u32, bicycle.items.len) catch unreachable;
        const car_arr: []u32 = std.heap.wasm_allocator.alloc(u32, car.items.len) catch unreachable;
        const rail_arr: []u32 = std.heap.wasm_allocator.alloc(u32, rail.items.len) catch unreachable;
        const other_arr: []u32 = std.heap.wasm_allocator.alloc(u32, other.items.len) catch unreachable;
        const building_arr: []u32 = std.heap.wasm_allocator.alloc(u32, building.items.len) catch unreachable;

        @memcpy(foot_arr, foot.items);
        @memcpy(bicycle_arr, bicycle.items);
        @memcpy(car_arr, car.items);
        @memcpy(rail_arr, rail.items);
        @memcpy(other_arr, other.items);
        @memcpy(building_arr, building.items);

        print("Sizes: foot: {} bicycle: {} car: {} rail: {} other: {} building: {}", .{ foot.items.len, bicycle.items.len, car.items.len, rail.items.len, other.items.len, building.items.len });
        return .{
            .foot = foot_arr,
            .bicycle = bicycle_arr,
            .car = car_arr,
            .rail = rail_arr,
            .other = other_arr,
            .building = building_arr,
        };
    }
};

const Renderer = struct {
    program: i32,
    vao: i32,
    lat_center: FloatUniform,
    lon_center: FloatUniform,
    tX: f32,
    tY: f32,
    zPos: f32,
    rX: f32,
    rY: f32,
    rZ: f32,
    viewAngle: f32,
    near: f32,
    far: f32,
    aspect: FloatUniform,
    zoom: FloatUniform,
    colour: VecUniform,
    height: FloatUniform,
    roofHeight: f32,
    projection: Mat4Uniform,
    transformation: Mat4Uniform,
    num_line_segments: usize,
    foot: []u32,
    bicycle: []u32,
    car: []u32,
    rail: []u32,
    other: []u32,
    building: []u32,

    pub fn init(aspect_val: f32, map_data: []const u8, metadata: Metadata) Renderer {
        // Now create an array of positions for the square.
        const program = js.compileLinkProgram(vs_source, vs_source.len, fs_source, fs_source.len);
        //const roof_program = js.compileLinkProgram(rs_source, rs_source.len, fs_source, fs_source.len);

        const point_data: []const f32 = @alignCast(std.mem.bytesAsSlice(f32, map_data[0..@intCast(metadata.n_nodes * 12)]));
        const vao = js.bind2DFloat32Data(point_data.ptr, point_data.len);

        const lat_center = FloatUniform.init(program, "lat_center", (metadata.max_lat + metadata.min_lat) / 2.0);
        //const lat_center_roof = FloatUniform.init(roof_program, "lat_center", (metadata.max_lat + metadata.min_lat) / 2.0);
        const lon_center = FloatUniform.init(program, "lon_center", (metadata.max_lon + metadata.min_lon) / 2.0);

        print("Lat center: {}", .{lat_center.val});
        print("Lon center: {}", .{lon_center.val});

        print("Left: {d}", .{metadata.min_lon - lon_center.val});
        print("Bounds: L{d} R{d} T{d} B{d}", .{ metadata.min_lon - lon_center.val, metadata.max_lon - lon_center.val, metadata.max_lat - lat_center.val, metadata.min_lat - lat_center.val });

        //const lon_center_roof = FloatUniform.init(roof_program, "lon_center", (metadata.max_lon + metadata.min_lon) / 2.0);
        //const zoom = FloatUniform.init(program, "zoom", 2.0 / (metadata.max_lon - metadata.min_lon));
        const zoom = FloatUniform.init(program, "zoom", 1.0);
        //const zoom_roof = FloatUniform.init(roof_program, "zoom", 2.0 / (metadata.max_lon - metadata.min_lon));

        const aspect = FloatUniform.init(program, "aspect", aspect_val);
        //const aspect_roof = FloatUniform.init(roof_program, "aspect", aspect_val);

        const colour = VecUniform.init(program, "vColor", .{ 1.0, 1.0, 1.0, 1.0 });

        const height = FloatUniform.init(program, "height", 0.0);

        const viewAngle: f32 = 45;

        const near: f32 = 0.011;
        const far: f32 = 0.1813;

        const scale = near * std.math.tan((viewAngle / 2.0) * (std.math.pi / 180.0));

        const right = scale * aspect_val;
        const left = -right;
        const top = scale;
        const bottom = -top;

        const projVec: [16]f32 = .{
            (2 * near) / (right - left),     0,                               0,                                0,
            0,                               (2 * near) / (top - bottom),     0,                                0,
            (right + left) / (right - left), (top + bottom) / (top - bottom), -(far + near) / (far - near),     -1,
            0,                               0,                               (-2 * far * near) / (far - near), 0,
        };

        const projMat = Mat4.initRowMajor(projVec);

        const projection = Mat4Uniform.init(program, "projection", projMat.getColMajor());

        const transformation = Mat4Uniform.init(program, "transformation", undefined);

        const index_data: []const u32 = @alignCast(std.mem.bytesAsSlice(u32, map_data[@intCast(metadata.n_nodes * 12)..]));
        print("Index start: {x}", .{index_data[0..40]});
        const element_data = element_types.processData(index_data, &metadata);
        const zPos: f32 = -0.88;
        const tX: f32 = 0.0;
        const tY: f32 = 0.0;
        js.reportFOV(viewAngle);
        js.reportTranslation(tX, tY, zPos);
        js.reportRotation(0.0, 0.0, 0.0);
        print("Zoom: {d}", .{zoom.val});
        print("Translation: {d} {d} {d}", .{ tX, tY, zPos });
        print("Rotation: {d} {d} {d}", .{ 45, 0, 0 });

        const roofHeight: f32 = 0.01;

        return .{
            .program = program,
            //.roof_program = roof_program,
            .vao = vao,
            .lat_center = lat_center,
            .lon_center = lon_center,
            .tX = tX,
            .tY = tY,
            .zPos = zPos,
            .rX = 45.0,
            .rY = 0.0,
            .rZ = 0.0,
            .near = near,
            .far = far,
            .viewAngle = viewAngle,
            .aspect = aspect,
            .zoom = zoom,
            .colour = colour,
            .height = height,
            .roofHeight = roofHeight,
            .projection = projection,
            .transformation = transformation,
            .num_line_segments = index_data.len,
            .foot = element_data.foot,
            .bicycle = element_data.bicycle,
            .car = element_data.car,
            .rail = element_data.rail,
            .other = element_data.other,
            .building = element_data.building,
        };
    }

    pub fn setAspect(self: *Renderer, aspect: f32) void {
        self.aspect.val = aspect;
        self.render();
    }

    fn renderLayer(self: *Renderer, index_data: []u32, colour: [4]f32, update_elements: bool) void {
        if (index_data.len == 0) {
            return;
        }
        //print("Binding foot", .{});
        if (update_elements) {
            _ = js.bindEbo(index_data.ptr, index_data.len);
        }

        self.colour.val = colour;
        self.colour.set();
        js.glDrawElements(Gl.LINE_STRIP, @intCast(index_data.len), Gl.UNSIGNED_INT, 0);
    }

    pub fn makeProjection(near: f32, far: f32, aspect: f32, fovy: f32) Mat4 {
        const scale = near * std.math.tan((fovy / 2.0) * (std.math.pi / 180.0));
        const right = scale * aspect;
        const left = -right;
        const top = scale;
        const bottom = -top;

        const projVec: [16]f32 = .{
            (2 * near) / (right - left),     0,                               0,                                0,
            0,                               (2 * near) / (top - bottom),     0,                                0,
            (right + left) / (right - left), (top + bottom) / (top - bottom), -(far + near) / (far - near),     -1,
            0,                               0,                               (-2 * far * near) / (far - near), 0,
        };

        const projMat = Mat4.initRowMajor(projVec);

        return projMat;
    }

    pub fn makeTransformation(lon_center: f32, lat_center: f32, x: f32, y: f32, z: f32, rX: f32, rY: f32, rZ: f32) Mat4 {
        const rXr = rX * std.math.pi / 180.0;
        const rYr = rY * std.math.pi / 180.0;
        const rZr = rZ * std.math.pi / 180.0;
        const cX: f32 = std.math.cos(rXr);
        const sX: f32 = std.math.sin(rXr);
        const cY: f32 = std.math.cos(rYr);
        const sY: f32 = std.math.sin(rYr);
        const cZ: f32 = std.math.cos(rZr);
        const sZ: f32 = std.math.sin(rZr);

        const coordTranslation = Mat4.initRowMajor(.{
            1, 0, 0, lon_center,
            0, 1, 0, lat_center,
            0, 0, 1, 0,
            0, 0, 0, 1,
        });

        const translation = Mat4.initRowMajor(.{
            1, 0, 0, x,
            0, 1, 0, y,
            0, 0, 1, z,
            0, 0, 0, 1,
        });

        const rotationX = Mat4.initRowMajor(.{
            1, 0,   0,  0,
            0, cX,  sX, 0,
            0, -sX, cX, 0,
            0, 0,   0,  1,
        });
        const rotationY = Mat4.initRowMajor(.{
            cY, 0, -sY, 0,
            0,  1, 0,   0,
            sY, 0, cY,  0,
            0,  0, 0,   1,
        });

        const rotationZ = Mat4.initRowMajor(.{
            cZ, -sZ, 0, 0,
            sZ, cZ,  0, 0,
            0,  0,   1, 0,
            0,  0,   0, 1,
        });

        // const scaleMatrix = Mat4.initRowMajor(.{
        //     scale, 0,     0,     0,
        //     0,     scale, 0,     0,
        //     0,     0,     scale, 0,
        //     0,     0,     0,     1,
        // });

        // const stage1 = Mat4.multiply(translation, rotationX);
        // const stage2 = Mat4.multiply(stage1, rotationY);
        // const stage3 = Mat4.multiply(stage2, rotationZ);
        // const result = Mat4.multiply(stage3, scaleMatrix);

        // const stage1 = Mat4.multiply(translation, scaleMatrix);
        // const stage2 = Mat4.multiply(stage1, rotationX);
        // const stage3 = Mat4.multiply(stage2, rotationY);
        // const result = Mat4.multiply(stage3, rotationZ);

        //const stage1 = Mat4.multiply(scaleMatrix, rotationX);
        const stage1 = Mat4.multiply(translation, rotationX);
        const stage2 = Mat4.multiply(stage1, rotationY);
        const stage3 = Mat4.multiply(stage2, rotationZ);
        const result = Mat4.multiply(stage3, coordTranslation);

        return result;
    }
    pub fn setProjection(self: *Renderer) void {
        self.projection.val = Renderer.makeProjection(self.near, self.far, self.aspect.val, self.viewAngle).getColMajor();
        self.projection.set();
    }

    pub fn render(self: *Renderer) void {
        print("Zoom: {d}", .{self.zoom.val});
        print("Translation: {d} {d} {d}", .{ self.lon_center.val, self.lat_center.val, self.zPos });
        print("Rotation: {d} {d} {d}", .{ self.rX, self.rY, self.rZ });
        js.glBindVertexArray(self.vao);
        js.glClearColor(0.0, 0.0, 0.0, 1.0);
        js.glClear(Gl.COLOR_BUFFER_BIT);
        js.glUseProgram(self.program);
        //js.glUseProgram(self.program);
        //var isRoof = IUniform.init(self.program, "isBuildingRoof", 0);
        //isRoof.set();
        setUniforms(&.{ self.lat_center, self.lon_center, self.zoom, self.aspect });
        const scaledX = self.tX / (180 / self.viewAngle);
        const scaledY = self.tY / (180 / self.viewAngle);
        const transMat = Renderer.makeTransformation(-1.0 * self.lon_center.val, -1.0 * self.lat_center.val, -1 * scaledX, -1 * scaledY, self.zPos, self.rX, self.rY, self.rZ);
        self.transformation.val = transMat.getColMajor();

        self.setProjection();
        print("Transform Matrix Row-Major (Maths style):", .{});
        transMat.printRowMajor();

        self.projection.set();
        self.transformation.set();
        self.height.val = 0.0;
        self.height.set();

        self.renderLayer(self.other, .{ 1.0, 1.0, 1.0, 1.0 }, true);

        self.renderLayer(self.rail, .{ 0.0, 0.85, 1.0, 1.0 }, true);

        self.renderLayer(self.foot, .{ 0.0, 1.0, 0.0, 1.0 }, true);

        self.renderLayer(self.car, .{ 1.0, 1.0, 0.0, 1.0 }, true);

        self.renderLayer(self.bicycle, .{ 1.0, 0.0, 1.0, 1.0 }, true);

        self.renderLayer(self.building, .{ 1.0, 0.0, 0.0, 1.0 }, true);

        var greenVal: f32 = 0.0;
        const nLayers: f32 = 50.0;
        while (self.height.val < self.roofHeight) {
            self.height.val += self.roofHeight / nLayers;
            greenVal += 0.5 / nLayers;
            self.height.set();
            self.renderLayer(self.building, .{ 1.0, greenVal, 0.0, 0.75 }, false);
        }

        //self.height.val = 0.00004;
        //self.height.set();
        //self.renderLayer(self.building, .{ 1.0, 0.0, 1.0, 0.75 }, false);
        //self.height.val = 0.00006;
        //self.height.set();
        //self.renderLayer(self.building, .{ 1.0, 0.0, 0.0, 1.0 }, false);
    }
};
