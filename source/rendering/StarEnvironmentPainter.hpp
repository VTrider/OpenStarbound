#pragma once

#include "StarParallax.hpp"
#include "StarWorldRenderData.hpp"
#include "StarAssetTextureGroup.hpp"
#include "StarRenderer.hpp"
#include "StarWorldCamera.hpp"
#include "StarPerlin.hpp"
#include "StarRandomPoint.hpp"

namespace Star {

STAR_CLASS(EnvironmentPainter);

class EnvironmentPainter {
public:
  EnvironmentPainter(V2::RendererPtr renderer);

  void update(float dt);

  void renderStars(float pixelRatio, Vec2F const& screenSize, SkyRenderData const& sky);
  void renderStarsV2(float pixelRatio, Vec2F const& screenSize, SkyRenderData const& sky);
  void renderDebrisFields(float pixelRatio, Vec2F const& screenSize, SkyRenderData const& sky);
  void renderBackOrbiters(float pixelRatio, Vec2F const& screenSize, SkyRenderData const& sky);
  void renderPlanetHorizon(float pixelRatio, Vec2F const& screenSize, SkyRenderData const& sky);
  void renderFrontOrbiters(float pixelRatio, Vec2F const& screenSize, SkyRenderData const& sky);
  void renderSky(Vec2F const& screenSize, SkyRenderData const& sky);

  void renderParallaxLayers(Vec2F parallaxWorldPosition, WorldCamera const& camera, ParallaxLayers const& layers, SkyRenderData const& sky);

  void cleanup(int64_t textureTimeout);

private:
  class StarInstanceData {
  public:
    Vec2F position;
    float frameOffset;
    uint16_t type;
  };

  static float const SunriseTime;
  static float const SunsetTime;
  static float const SunFadeRate;
  static float const MaxFade;
  static float const RayPerlinFrequency;
  static float const RayPerlinAmplitude;
  static int const RayCount;
  static float const RayMinWidth;
  static float const RayWidthVariance;
  static float const RayAngleVariance;
  static float const SunRadius;
  static float const RayColorDependenceLevel;
  static float const RayColorDependenceScale;
  static float const RayUnscaledAlphaVariance;
  static float const RayMinUnscaledAlpha;
  static Vec3B const RayColor;

  void drawRays(float pixelRatio, SkyRenderData const& sky, Vec2F start, float length, double time, float alpha);
  void drawRay(float pixelRatio,
      SkyRenderData const& sky,
      Vec2F start,
      float width,
      float length,
      float angle,
      double time,
      Vec3B color,
      float alpha);
  void drawOrbiter(float pixelRatio, Vec2F const& screenSize, SkyRenderData const& sky, SkyOrbiter const& orbiter);

  uint64_t starsHash(SkyRenderData const& sky, Vec2F const& viewSize) const;
  void setupStars(SkyRenderData const& sky);
  void setupStarsV2(SkyRenderData const& sky);

  V2::RendererPtr m_renderer;
  AssetTextureGroupPtr m_textureGroup;

  double m_timer;
  PerlinF m_rayPerlin;

  uint64_t m_starsHash{};
  List<TexturePtr> m_starTextures;
  List<V2::PooledTexturePtr> m_starTexturesV2;
  shared_ptr<Random2dPointGenerator<pair<size_t, float>>> m_starGenerator;
  List<shared_ptr<Random2dPointGenerator<pair<String, float>, double>>> m_debrisGenerators;

  V2::CommandBuffer m_starsDrawCmd;
  V2::PipelineDescriptor m_starsRender;
  V2::PipelineDescriptor m_starsGenerator;
  V2::DescriptorSet m_starsDescriptorSet;

  struct StarInstance {
    Mat3F transform;
    uint32_t textureIndex;
  };
};

}
