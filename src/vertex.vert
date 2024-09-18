#version 300 es
precision highp float;

in vec4 aVertexPosition;
uniform float lat_center;
uniform float lon_center;
uniform float zoom;
uniform float aspect;
uniform float height;
uniform mat4 projection;
uniform mat4 transformation;
out float normHeight;


void main() {

  float normHeight1 = aVertexPosition.z;
  if (normHeight1 > 80.0) {
    normHeight1 = 80.0;
  }

  
  vec4 adjustedVertexPosition = aVertexPosition;
  adjustedVertexPosition.z *= height;

  normHeight = normHeight1;
  gl_Position = projection * transformation * adjustedVertexPosition;

  gl_PointSize =0.5;

}
