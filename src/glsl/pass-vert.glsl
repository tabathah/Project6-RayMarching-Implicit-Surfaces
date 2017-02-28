varying vec2 f_uv;
varying mat4 viewMat;
varying mat4 projMat;

void main() {
    f_uv = uv;
    viewMat = modelViewMatrix;
    projMat = projectionMatrix;
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}