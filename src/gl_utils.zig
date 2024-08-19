const gui = @import("gui_bindings.zig");
const lin = @import("lin.zig");
const MapPos = lin.Point;

pub const ViewState = struct {
    center: MapPos,
    zoom: f32,
    aspect: f32,
};

pub const FloatUniform = struct {
    loc: i32,

    pub fn init(program: i32, key: []const u8) FloatUniform {
        const loc = gui.glGetUniformLoc(program, key.ptr, key.len);
        return .{
            .loc = loc,
        };
    }

    pub fn set(self: *const FloatUniform, val: f32) void {
        gui.glUniform1f(self.loc, val);
    }

    pub fn set2(self: *const FloatUniform, a: f32, b: f32) void {
        gui.glUniform2f(self.loc, a, b);
    }
};

pub const IntUniform = struct {
    loc: i32,

    pub fn init(program: i32, key: []const u8) IntUniform {
        const loc = gui.glGetUniformLoc(program, key.ptr, key.len);
        return .{
            .loc = loc,
        };
    }

    pub fn set(self: *const IntUniform, val: i32) void {
        gui.glUniform1i(self.loc, val);
    }
};

pub const Gl = struct {
    // https://registry.khronos.org/webgl/specs/latest/1.0/
    pub const COLOR_BUFFER_BIT = 0x00004000;
    pub const POINTS = 0x0000;
    pub const TEXTURE0 = 0x84C0;
    pub const TRIANGLE_STRIP = 0x0005;
    pub const LINE_STRIP = 0x0003;
    pub const UNSIGNED_BYTE = 0x1401;
    pub const UNSIGNED_INT = 0x1405;
    pub const ARRAY_BUFFER = 0x8892;
    pub const ELEMENT_ARRAY_BUFFER = 0x8893;
    pub const STATIC_DRAW = 0x88E4;
    pub const FLOAT = 0x1406;
    pub const TEXTURE_2D = 0x0DE1;
    pub const RGBA = 0x1908;
    pub const TEXTURE_MAG_FILTER = 0x2800;
    pub const TEXTURE_MIN_FILTER = 0x2801;
    pub const TEXTURE_WRAP_S = 0x2802;
    pub const TEXTURE_WRAP_T = 0x2803;
    pub const REPEAT = 0x2901;
    pub const LINEAR = 0x2601;
    pub const RGBA32F = 0x8814;
};
