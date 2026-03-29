#version 460 core

#extension GL_ARB_bindless_texture : require

layout (std430, location = 2) readonly buffer TexturePool {
  sampler2D texturePool[];
};

layout (location = 0) in vec2 uv;
layout (location = 1) in flat uint texPoolIndex;

layout (location = 0) out vec4 fragColor;

void main() {
  vec4 texColor = texture(texturePool[texPoolIndex], uv);

  if (texColor.a <= 0)
    discard;

  fragColor = texColor;
}
