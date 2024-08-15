// FIXME: Prune imports
const std = @import("std");
const Allocator = std.mem.Allocator;
const gui = @import("gui_bindings.zig");
const gl_utils = @import("gl_utils.zig");
const ViewState = gl_utils.ViewState;
const FloatUniform = gl_utils.FloatUniform;
const IntUniform = gl_utils.IntUniform;
const Gl = gl_utils.Gl;
const map_data = @import("map_data.zig");
const ClosestStopLookup = @import("ClosestStopLookup.zig");

program: i32,
vao: i32,
vbo: i32,
tex: i32,
lat_center: FloatUniform,
lon_center: FloatUniform,
aspect: FloatUniform,
zoom: FloatUniform,
texture: gl_utils.IntUniform,
point_size: FloatUniform,
width: FloatUniform,
height: FloatUniform,
x_samples: gl_utils.IntUniform,
y_samples: gl_utils.IntUniform,

const Renderer = @This();

pub fn init(metadata: map_data.MeterMetadata) Renderer {
    // Now create an array of positions for the square.
    const program = gui.compileLinkProgram(vs_source, vs_source.len, fs_source, fs_source.len);

    const buffers = setupBuffers(metadata);
    const tex = setupTexture();

    const lat_center = FloatUniform.init(program, "lat_center");
    const lon_center = FloatUniform.init(program, "lon_center");
    const zoom = FloatUniform.init(program, "zoom");
    const aspect = FloatUniform.init(program, "aspect");
    const point_size = FloatUniform.init(program, "point_size");
    const width = FloatUniform.init(program, "width");
    const height = FloatUniform.init(program, "height");
    const x_samples = IntUniform.init(program, "x_samples");
    const y_samples = IntUniform.init(program, "y_samples");
    const texture = IntUniform.init(program, "u_texture");

    gui.glUseProgram(program);
    width.set(metadata.width);
    height.set(metadata.height);

    return .{
        .program = program,
        .vao = buffers.vao,
        .vbo = buffers.vbo,
        .tex = tex,
        .lat_center = lat_center,
        .lon_center = lon_center,
        .aspect = aspect,
        .texture = texture,
        .zoom = zoom,
        .point_size = point_size,
        .x_samples = x_samples,
        .y_samples = y_samples,
        .width = width,
        .height = height,
    };
}

pub fn feed(self: *Renderer, alloc: Allocator, closest_stop_lookup: ClosestStopLookup) !void {
    const vertex_buf = try alloc.alloc(f32, closest_stop_lookup.storage.len * 4);
    defer alloc.free(vertex_buf);

    var sample_it = closest_stop_lookup.sampleIt();
    var i: usize = 0;
    while (sample_it.next()) |item| {
        vertex_buf[i] = item.x;
        vertex_buf[i + 1] = item.y;
        vertex_buf[i + 2] = item.closest_point.x;
        vertex_buf[i + 3] = item.closest_point.y;
        i += 4;
    }

    gui.glBindTexture(Gl.TEXTURE_2D, self.tex);
    gui.glTexImage2D(Gl.TEXTURE_2D, 0, Gl.RGBA32F, Gl.FLOAT, @ptrCast(vertex_buf.ptr), ClosestStopLookup.x_samples, ClosestStopLookup.y_samples);
    gui.glUseProgram(self.program);
    self.x_samples.set(ClosestStopLookup.x_samples);
    self.y_samples.set(ClosestStopLookup.y_samples);
}

pub fn deinit(self: *Renderer) void {
    gui.glDeleteBuffer(self.vbo);
    gui.glDeleteVertexArray(self.vao);
}

pub fn bind(self: *Renderer) BoundRenderer {
    gui.glBindVertexArray(self.vao);
    gui.glUseProgram(self.program);
    return .{
        .inner = self,
    };
}

fn setupTexture() i32 {
    const tex = gui.glCreateTexture();
    gui.glBindTexture(Gl.TEXTURE_2D, tex);
    gui.glTexParameteri(
        Gl.TEXTURE_2D,
        Gl.TEXTURE_WRAP_S,
        Gl.REPEAT,
    );

    gui.glTexParameteri(
        Gl.TEXTURE_2D,
        Gl.TEXTURE_WRAP_T,
        Gl.REPEAT,
    );
    gui.glTexParameteri(
        Gl.TEXTURE_2D,
        Gl.TEXTURE_MIN_FILTER,
        Gl.LINEAR,
    );
    gui.glTexParameteri(
        Gl.TEXTURE_2D,
        Gl.TEXTURE_MAG_FILTER,
        Gl.LINEAR,
    );

    return tex;
}

const Buffers = struct {
    vao: i32,
    vbo: i32,
};

fn setupBuffers(metadata: map_data.MeterMetadata) Buffers {
    const points: []const f32 = &.{
        0,              0,
        metadata.width, 0.0,
        0.0,            metadata.height,
        metadata.width, metadata.height,
    };

    const vao = gui.glCreateVertexArray();
    gui.glBindVertexArray(vao);

    const vbo = gui.glCreateBuffer();
    gui.glBindBuffer(Gl.ARRAY_BUFFER, vbo);
    gui.glBufferData(Gl.ARRAY_BUFFER, @ptrCast(points.ptr), points.len * 4, Gl.STATIC_DRAW);

    gui.glVertexAttribPointer(0, 2, Gl.FLOAT, false, 0, 0);
    gui.glEnableVertexAttribArray(0);

    return .{
        .vao = vao,
        .vbo = vbo,
    };
}

const vs_source = @embedFile("vertex_closest_stop.glsl");
const fs_source = @embedFile("fragment_closest_stop.glsl");

const BoundRenderer = struct {
    inner: *Renderer,

    pub fn render(self: *const BoundRenderer, view_state: ViewState) void {
        gui.glBindVertexArray(self.inner.vao);

        gui.glActiveTexture(Gl.TEXTURE0);
        gui.glBindTexture(Gl.TEXTURE_2D, self.inner.tex);
        self.inner.texture.set(0);

        self.inner.lat_center.set(view_state.center.y);
        self.inner.lon_center.set(view_state.center.x);
        self.inner.aspect.set(view_state.aspect);
        self.inner.zoom.set(view_state.zoom);
        self.inner.point_size.set(10.0);
        gui.glDrawArrays(Gl.TRIANGLE_STRIP, 0, 4);
    }
};
