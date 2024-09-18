#version 300 es

precision highp float;
uniform vec4 vColor;
in float normHeight;
out vec4 color;

void main() {

  vec4 newColor = vColor;
  if (normHeight > 0.0) {
      newColor.y = normHeight / 80.0;
  }

  color = newColor;
}
