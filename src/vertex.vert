#version 300 es
precision highp float;

in vec4 aVertexPosition;
//in vec4 aVertexColor;
uniform float lat_center;
uniform float lon_center;
uniform float zoom;
uniform float aspect;
uniform float height;
uniform mat4 projection;
uniform mat4 transformation;
//out vec4 vColor;

void main() {
  
  gl_Position = projection * transformation * aVertexPosition;
  //gl_Position.y += height;                   
  gl_Position.y -= lat_center; //49.10902;
  //gl_Position.y *= aspect * zoom;
  

  gl_Position.x -= lon_center; //123.50649;
  //gl_Position.x *= zoom;


  gl_PointSize =0.5;
  //vColor = aVertexColor;
}
