#pragma once

#include "StarDrawable.hpp"
#include "StarRenderer.hpp"
#include "StarAssetTextureGroup.hpp"

namespace Star {

STAR_CLASS(DrawablePainter);

class DrawablePainter {
public:
  DrawablePainter(V2::RendererPtr renderer, AssetTextureGroupPtr textureGroup);

  void drawDrawable(Drawable const& drawable);

  void cleanup(int64_t textureTimeout);

private:
  V2::RendererPtr m_renderer;
  AssetTextureGroupPtr m_textureGroup;
};

}
