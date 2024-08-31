#version 300 es

precision highp float;
uniform vec4 vColor;

out vec4 color;

void main() {
  color = vColor;
}
