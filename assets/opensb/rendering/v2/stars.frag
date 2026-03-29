#version 460 core

#extension GL_ARB_bindless_texture : require

layout (location = 0) in vec2 uv;
layout (location = 1) in flat sampler2D tex;

layout (location = 0) out vec4 fragColor;

void main() {
  vec4 texColor = texture(tex, uv);

  if (texColor.a <= 0)
    discard;

  fragColor = texColor;
}
