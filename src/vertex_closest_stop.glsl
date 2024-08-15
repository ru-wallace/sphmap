#version 300 es
precision highp float;

in vec2 vertex_pos;
uniform float lat_center;
uniform float lon_center;
uniform float zoom;
uniform float aspect;
uniform float point_size;
out vec2 frag_in;

void main() {
  gl_Position = vec4(vertex_pos, 0, 1);
  gl_Position.y -= lat_center;;
  gl_Position.y *= aspect * zoom;
  gl_Position.x -= lon_center;;
  gl_Position.x *= zoom;
  gl_PointSize = point_size;
  frag_in = vertex_pos;
}
