#version 300 es

precision highp float;
uniform float width;
uniform float height;
uniform int x_samples;
uniform int y_samples;
uniform sampler2D u_texture;
in vec2 frag_in;
out vec4 out_color;

float getOffs(int x, int y) {
  vec4 tex_val = texelFetch(u_texture, ivec2(x, y), 0);
  vec2 sample_pos = tex_val.xy;
  vec2 additional_offs = sample_pos - frag_in;
  vec2 sample_offs = tex_val.ba;
  return abs(length(sample_offs) + dot(additional_offs, normalize(sample_offs)));
}

void main() {
  float x = frag_in.x / width;
  float y = frag_in.y / height;

  float sample_space_x = x * (float(x_samples - 1));
  float sample_space_y = y * (float(y_samples - 1));
  float sample_space_top = ceil(sample_space_y);
  float sample_space_bottom = floor(sample_space_y);
  float sample_space_right = ceil(sample_space_x);
  float sample_space_left = floor(sample_space_x);

  float y_lerp = sample_space_y - sample_space_bottom;
  float x_lerp = sample_space_x - sample_space_left;

  if ((x_lerp > 0.95 || x_lerp < 0.05) && (y_lerp < 0.05 || y_lerp > 0.95)) {
    out_color = vec4(0.0, 0.0, 0.0, 1.0);
    return;
  }

  float tr = getOffs(int(ceil(sample_space_x)), int(ceil(sample_space_y)));
  float bl = getOffs(int(floor(sample_space_x)), int(floor(sample_space_y)));
  float br = getOffs(int(ceil(sample_space_x)), int(floor(sample_space_y)));
  float tl = getOffs(int(floor(sample_space_x)), int(ceil(sample_space_y)));

  float a = tr * x_lerp + tl * (1.0 - x_lerp);
  float b = br * x_lerp + bl * (1.0 - x_lerp);
  float dist = b * (1.0 - y_lerp) + a * y_lerp;

  float r2 = 1.0 / (1.0 + dist * 0.0040);
  float b2 = 1.0 - r2;
  out_color = vec4(r2, 0.0, b2, 0.8);
}
