class WasmHandler {
  constructor(gl) {
    this.gl = gl;
    this.ebos = [];
    this.vaos = [];
    this.programs = [];
    this.uniform_locs = [];
    this.memory = null;
  }

  compileLinkProgram(vs, vs_len, fs, fs_len) {
    const dec = new TextDecoder("utf8");
    const vs_source = dec.decode(
      new Uint8Array(this.memory.buffer, vs, vs_len),
    );
    const fs_source = dec.decode(
      new Uint8Array(this.memory.buffer, fs, fs_len),
    );


    const vertexShader = loadShader(this.gl, this.gl.VERTEX_SHADER, vs_source);
    const fragmentShader = loadShader(
      this.gl,
      this.gl.FRAGMENT_SHADER,
      fs_source,
    );

    const shaderProgram = this.gl.createProgram();
    this.gl.attachShader(shaderProgram, vertexShader);
    this.gl.attachShader(shaderProgram, fragmentShader);
    this.gl.linkProgram(shaderProgram);
    if (!this.gl.getProgramParameter(shaderProgram, this.gl.LINK_STATUS)) {
      alert(
        `Unable to initialize the shader program: ${this.gl.getProgramInfoLog(
          shaderProgram,
        )}`,
      );
    }

    this.programs.push(shaderProgram);
    return this.programs.length - 1;
  }

  bind2DFloat32Data(ptr, len) {
    const positions = new Float32Array(this.memory.buffer, ptr, len);
    const positionBuffer = this.gl.createBuffer();
    this.gl.bindBuffer(this.gl.ARRAY_BUFFER, positionBuffer);
    this.gl.bufferData(this.gl.ARRAY_BUFFER, positions, this.gl.STATIC_DRAW);
    this.gl.vertexAttribPointer(0, 3, this.gl.FLOAT, true, 0, 0);
    this.gl.enableVertexAttribArray(0);
    //this.gl.vertexAttribPointer(1, 4, this.gl.FLOAT, false, 24, 8);
    //this.gl.enableVertexAttribArray(1);
  }




  bindEbo(ptr, len) {
    //console.log("bindEbo ptr: " + ptr + " len: " + len);
    const indices = new Uint32Array(this.memory.buffer, ptr, len);
    //console.log(indices.slice(0, 30));
    //console.log(indices.length)
    const ebo = this.gl.createBuffer();
    this.gl.bindBuffer(this.gl.ELEMENT_ARRAY_BUFFER, ebo);
    this.gl.bufferData(
      this.gl.ELEMENT_ARRAY_BUFFER,
      indices,
      this.gl.STATIC_DRAW,
    );
    this.ebos.push(ebo);
    //return this.ebos.length - 1;
  }

  unbindEbo() {
    this.ebos.pop();
  }

  getUniformLocWasm(program, namep, name_len) {
    const name_data = new Uint8Array(this.memory.buffer, namep, name_len);
    const name = new TextDecoder("utf8").decode(name_data);
    this.uniform_locs.push(
      this.gl.getUniformLocation(this.programs[program], name),
    );
    return this.uniform_locs.length - 1;
  }

  glBindVertexArray(vao) {
    this.gl.bindVertexArray(this.vaos[vao]);
  }

  glUseProgram(program) {
    this.gl.useProgram(this.programs[program]);
  }

  glUniform1f(loc, val) {
    this.gl.uniform1f(this.uniform_locs[loc], val);
  }
  glUniform4f(loc, val1, val2, val3, val4) {
    //console.log("Uniform4f: " + val1 + " " + val2 + " " + val3 + " " + val4);
    this.gl.uniform4f(this.uniform_locs[loc], val1, val2, val3, val4);
  }

  glUniform1i(loc, val) {
    this.gl.uniform1i(this.uniform_locs[loc], val);
  }

  

  glUniformMatrix4fv(loc, transpose, ptr) {
    
    const data = new Float32Array(this.memory.buffer, ptr, 16);
    //console.log("UniformMatrix4fv: Addr:" + ptr);
    //console.log(data);
    this.gl.uniformMatrix4fv(this.uniform_locs[loc], transpose, data);
  }

  logWasm(s, len) {
    const buf = new Uint8Array(this.memory.buffer, s, len);
    console.log(new TextDecoder("utf8").decode(buf));
  }


}

function initCanvas() {
  const canvas = document.getElementById("canvas");
  canvas.width = canvas.clientWidth;
  canvas.height = canvas.clientHeight;
  return canvas;
}

function loadShader(gl, type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);

  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    alert(
      `An error occurred compiling the shaders: ${gl.getShaderInfoLog(shader)}`,
    );
    gl.deleteShader(shader);
    return null;
  }

  return shader;
}



function updateFOV(zoomVal) {
  document.getElementById("fov").value = zoomVal;
  document.getElementById("fovVal").innerHTML = zoomVal;
}

function updateTranslation(tXVal, tYVal, tZVal) {
  document.getElementById("tXVal").innerHTML = tXVal; 
  document.getElementById("tX").value = tXVal;
  document.getElementById("tYVal").innerHTML = tYVal;
  document.getElementById("tY").value = tYVal;
  document.getElementById("tZVal").innerHTML = tZVal;
  document.getElementById("tZ").value = tZVal;
}
function updateRotation(rXVal, rYVal, rZVal) {
  document.getElementById("rXVal").innerHTML = rXVal;
  document.getElementById("rX").value
  document.getElementById("rYVal").innerHTML = rYVal;
  document.getElementById("rY").value
  document.getElementById("rZVal").innerHTML = rZVal;
  document.getElementById("rZ").value
}



async function instantiateWasmModule(wasm_handlers) {
  const wasmEnv = {
    env: {
      logWasm: wasm_handlers.logWasm.bind(wasm_handlers),
      compileLinkProgram: wasm_handlers.compileLinkProgram.bind(wasm_handlers),
      bind2DFloat32Data: wasm_handlers.bind2DFloat32Data.bind(wasm_handlers),
      bindEbo: wasm_handlers.bindEbo.bind(wasm_handlers),
      unbindEbo: wasm_handlers.unbindEbo.bind(wasm_handlers),
      glBindVertexArray: wasm_handlers.glBindVertexArray.bind(wasm_handlers),
      glClearColor: (...args) => wasm_handlers.gl.clearColor(...args),
      glClear: (...args) => wasm_handlers.gl.clear(...args),
      glUseProgram: wasm_handlers.glUseProgram.bind(wasm_handlers),
      glDrawArrays: (...args) => wasm_handlers.gl.drawArrays(...args),
      glDrawElements: (...args) => wasm_handlers.gl.drawElements(...args),
      glGetUniformLoc: wasm_handlers.getUniformLocWasm.bind(wasm_handlers),
      glUniform1f: wasm_handlers.glUniform1f.bind(wasm_handlers),
      glUniform4f: wasm_handlers.glUniform4f.bind(wasm_handlers),
      glUniform1i: wasm_handlers.glUniform1i.bind(wasm_handlers),
      glUniformMatrix4fv: wasm_handlers.glUniformMatrix4fv.bind(wasm_handlers),
      reportFOV: updateFOV.bind(wasm_handlers),
      reportTranslation: updateTranslation.bind(wasm_handlers),
      reportRotation: updateRotation.bind(wasm_handlers),


    },
  };

  const mod = await WebAssembly.instantiateStreaming(
    fetch("index.wasm"),
    wasmEnv,
  );
  wasm_handlers.memory = mod.instance.exports.memory;

  return mod;
}



async function loadPointsData(mod) {
  console.log("Loading points data");
  const map_data_response = await fetch("map_data.bin");
  const data_reader = map_data_response.body.getReader({
    mode: "byob",
  });
  let array_buf = new ArrayBuffer(16384);
  while (true) {
    const { value, done } = await data_reader.read(new Uint8Array(array_buf));
    if (done) break;
    console.log("Global_chunk value" + mod.instance.exports.global_chunk.value);
    array_buf = value.buffer;
    const chunk_buf = new Uint8Array(
      mod.instance.exports.memory.buffer,
      mod.instance.exports.global_chunk.value,
      16384,
    );
    chunk_buf.set(value);
    console.log("value length: " + value.length);
    mod.instance.exports.pushMapData(value.length);
    
  }
}

async function loadElementData(mod) {
   console.log("Loading element data");
  const map_data_response = await fetch("element_data.bin");
  const data_reader = map_data_response.body.getReader({
    mode: "byob",
  });
  let array_buf = new ArrayBuffer(16384);
  while (true) {
    const { value, done } = await data_reader.read(new Uint8Array(array_buf));
    if (done) break;
    console.log("Global_chunk value" + mod.instance.exports.global_chunk.value);
    array_buf = value.buffer;
    const chunk_buf = new Uint8Array(
      mod.instance.exports.memory.buffer,
      mod.instance.exports.global_chunk.value,
      16384,
    );

    console.log("Element data: " + value.slice(0, 30).toString(16));
    chunk_buf.set(value);
    mod.instance.exports.pushMapData(value.length);
  }
}

async function loadMetadata(mod) {
  console.log("Loading metadata");
  const map_data_response = await fetch("map_data.json");
  const data = await map_data_response.arrayBuffer();
  console.log("Global_chunk value" + mod.instance.exports.global_chunk.value);
  const chunk_buf = new Uint8Array(
    mod.instance.exports.memory.buffer,
    mod.instance.exports.global_chunk.value,
    data.byteLength,
  );

  chunk_buf.set(new Uint8Array(data));
  console.log("value length: " + data.byteLength);
  mod.instance.exports.setMetadata(data.byteLength);
}

function canvasAspect(canvas) {
  const rect = canvas.getBoundingClientRect();
  return rect.width / rect.height;
}

class CanvasInputHandler {
  constructor(canvas, mod) {
    this.canvas = canvas;
    this.mod = mod;
  }

  normX(client_x) {
    const rect = this.canvas.getBoundingClientRect();
    return (client_x - rect.left) / rect.width;
  }

  normY(client_y) {
    const rect = this.canvas.getBoundingClientRect();
    return (client_y - rect.top) / rect.height;
  }

  onMouseDown(ev) {
    this.mod.instance.exports.mouseDown(
      this.normX(ev.clientX),
      this.normY(ev.clientY),
    );
  }

  onMouseMove(ev) {
    this.mod.instance.exports.mouseMove(
      this.normX(ev.clientX),
      this.normY(ev.clientY),
    );
  }

  onMouseUp() {
    this.mod.instance.exports.mouseUp();
  }

  onWheel(ev) {
    this.mod.instance.exports.zoom(ev.deltaY);
  }

  onResize() {
    this.mod.instance.exports.setAspect(canvasAspect(this.canvas));
  }

  setCanvasCallbacks() {
    this.canvas.onmousedown = this.onMouseDown.bind(this);
    window.onmouseup = this.onMouseUp.bind(this);
    this.canvas.onmousemove = this.onMouseMove.bind(this);
    this.canvas.onwheel = this.onWheel.bind(this);
    window.onresize = this.onResize.bind(this);
  }
}

function makeGl(canvas) {
  const gl = canvas.getContext("webgl2");
  gl.enable(gl.DEPTH_TEST);
  gl.enable(gl.CULL_FACE);

  if (gl === null) {
    throw new Error("Failed to initialize gl context");
  }

  return gl;
}

async function init() {
  const canvas = initCanvas();
  const wasm_handlers = new WasmHandler(makeGl(canvas));
  const mod = await instantiateWasmModule(wasm_handlers);
  await loadPointsData(mod);
  //await loadElementData(mod);
  await loadMetadata(mod);

  mod.instance.exports.init(canvasAspect(canvas));
  //mod.instance.exports.render();

  const canvas_callbacks = new CanvasInputHandler(canvas, mod);
  canvas_callbacks.setCanvasCallbacks();

  const tXSlider = document.getElementById("tX");
  const tYSlider = document.getElementById("tY");
  const tZSlider = document.getElementById("tZ");
  const rXSlider = document.getElementById("rX");
  const rYSlider = document.getElementById("rY");
  const rZSlider = document.getElementById("rZ");


  const viewAngleSlider = document.getElementById("fov");
  const nearSlider = document.getElementById("near");
  const farSlider = document.getElementById("far");

  const roofZSlider = document.getElementById("roof");


  tXSlider.oninput = () => {
    document.getElementById("tXVal").innerHTML = tXSlider.value;
    mod.instance.exports.setTranslation(tXSlider.value, tYSlider.value, tZSlider.value);
    mod.instance.exports.render();
  }
  tYSlider.oninput = () => {
    document.getElementById("tYVal").innerHTML = tYSlider.value;
    mod.instance.exports.setTranslation(tXSlider.value, tYSlider.value, tZSlider.value);
    mod.instance.exports.render();
  }
  tZSlider.oninput = () => {
    document.getElementById("tZVal").innerHTML = tZSlider.value;
    mod.instance.exports.setTranslation(tXSlider.value, tYSlider.value, tZSlider.value);
    mod.instance.exports.render();
  }
  rXSlider.oninput = () => {
    document.getElementById("rXVal").innerHTML = rXSlider.value;
    mod.instance.exports.setRotation(rXSlider.value, rYSlider.value, rZSlider.value);
    mod.instance.exports.render();
  }
  rYSlider.oninput = () => {
    document.getElementById("rYVal").innerHTML = rYSlider.value;
    mod.instance.exports.setRotation(rXSlider.value, rYSlider.value, rZSlider.value);
    mod.instance.exports.render();
  }
  rZSlider.oninput = () => {
    document.getElementById("rZVal").innerHTML = rZSlider.value;
    mod.instance.exports.setRotation(rXSlider.value, rYSlider.value, rZSlider.value);
    mod.instance.exports.render();
  }

  viewAngleSlider.oninput = () => {
    document.getElementById("fovVal").innerHTML = viewAngleSlider.value;
    mod.instance.exports.setViewAngle(viewAngleSlider.value);
    mod.instance.exports.render();
  }

  nearSlider.oninput = () => {
    document.getElementById("nearFarRatio").innerHTML = nearSlider.value / farSlider.value;
    document.getElementById("nearVal").innerHTML = nearSlider.value;
    mod.instance.exports.setNearFar(nearSlider.value, farSlider.value);
    mod.instance.exports.render();
  }

  farSlider.oninput = () => {
    document.getElementById("farVal").innerHTML = farSlider.value;
    mod.instance.exports.setNearFar(nearSlider.value, farSlider.value);
    mod.instance.exports.render();
  }

  roofZSlider.oninput = () => {
    document.getElementById("roofVal").innerHTML = roofZSlider.value;
    mod.instance.exports.setRoofHeight(roofZSlider.value);
    mod.instance.exports.render();
  }

  //updateMats();
  mod.instance.exports.render();


}

window.onload = init;
