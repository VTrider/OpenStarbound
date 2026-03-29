#version 460 core

#extension GL_ARB_bindless_texture : require

struct VertexAttribute {
  vec4 pos;
  vec2 uv;
};

struct StarInstance {
  sampler2D textureHandle;
  float m[9];
};

layout (std430, binding = 0) readonly buffer VertexAttributes {
  VertexAttribute[] vertices;
};

layout (std430, binding = 1) readonly buffer InstanceData {
  StarInstance instanceData[];
};

layout (location = 0) uniform vec2 screenSize;

layout (location = 1) out vec2 uv;
layout (location = 1) out flat sampler2D tex;

void main() {
  StarInstance star = instanceData[gl_InstanceID];

  VertexAttribute vertex = vertices[gl_VertexID];
  uv = vertex.uv;
  tex = star.textureHandle;

  mat3 transform = mat3(
      star.m[0], star.m[1], star.m[2],
      star.m[3], star.m[4], star.m[5],
      star.m[6], star.m[7], star.m[8]
  );

  vec2 screenPos = (transform * vertex.pos.xyz).xy;
  gl_Position = vec4(screenPos / screenSize * 2.0 - 1.0, 0.0, 1.0);
}