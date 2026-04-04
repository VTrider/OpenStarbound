#pragma once

#include "StarAssetPath.hpp"
#include "StarTextureAtlas.hpp"
#include "StarRenderer.hpp"

#include "GL/glew.h"

#include "array"
#include "string"
#include "unordered_map"

namespace Star {
 
namespace V2 {
class OpenGlRenderer;
}

STAR_CLASS(OpenGlRenderer);

constexpr size_t FrameBufferCount = 1;

// OpenGL 2.0 implementation of Renderer.  OpenGL context must be created and
// active during construction, destruction, and all method calls.
class OpenGlRenderer : virtual public Renderer {
public:
  OpenGlRenderer();
  ~OpenGlRenderer();

  String rendererId() const override;
  Vec2U screenSize() const override;

  void loadConfig(Json const& config) override;
  void loadEffectConfig(String const& name, Json const& effectConfig, StringMap<String> const& shaders) override;

  void setEffectParameter(String const& parameterName, RenderEffectParameter const& parameter) override;
  void setEffectScriptableParameter(String const& effectName, String const& parameterName, RenderEffectParameter const& parameter) override;
  Maybe<RenderEffectParameter> getEffectScriptableParameter(String const& effectName, String const& parameterName) override;
  Maybe<VariantTypeIndex> getEffectScriptableParameterType(String const& effectName, String const& parameterName) override;
  void setEffectTexture(String const& textureName, ImageView const& image) override;

  void setScissorRect(Maybe<RectI> const& scissorRect) override;

  bool switchEffectConfig(String const& name) override;

  virtual TexturePtr createTexture(Image const& texture, TextureAddressing addressing, TextureFiltering filtering) override;
  void setSizeLimitEnabled(bool enabled) override;
  void setMultiTexturingEnabled(bool enabled) override;
  void setMultiSampling(unsigned multiSampling) override;
  TextureGroupPtr createTextureGroup(TextureGroupSize size, TextureFiltering filtering) override;
  RenderBufferPtr createRenderBuffer() override;

  List<RenderPrimitive>& immediatePrimitives() override;
  void render(RenderPrimitive primitive) override;
  void renderBuffer(RenderBufferPtr const& renderBuffer, Mat3F const& transformation) override;

  void flush(Mat3F const& transformation) override;

  void setScreenSize(Vec2U screenSize);

  virtual void startFrame();
  virtual void finishFrame();

public:
  struct GlTextureAtlasSet : public TextureAtlasSet<GLuint> {
  public:
    GlTextureAtlasSet(unsigned atlasNumCells);

    GLuint createAtlasTexture(Vec2U const& size, PixelFormat pixelFormat) override;
    void destroyAtlasTexture(GLuint const& glTexture) override;
    void copyAtlasPixels(GLuint const& glTexture, Vec2U const& bottomLeft, Image const& image) override;

    TextureFiltering textureFiltering;
  };

  struct GlTextureGroup : enable_shared_from_this<GlTextureGroup>, public TextureGroup {
    GlTextureGroup(unsigned atlasNumCells);
    ~GlTextureGroup();

    TextureFiltering filtering() const override;
    TexturePtr create(Image const& texture) override;

    GlTextureAtlasSet textureAtlasSet;
  };

  friend class V2::OpenGlRenderer;

  struct GlTexture : virtual public Texture {
    virtual GLuint glTextureId() const = 0;
    virtual Vec2U glTextureSize() const = 0;
    virtual Vec2U glTextureCoordinateOffset() const = 0;
  };

  struct GlGroupedTexture : public GlTexture {
    ~GlGroupedTexture();

    Vec2U size() const override;
    TextureFiltering filtering() const override;
    TextureAddressing addressing() const override;

    GLuint glTextureId() const override;
    Vec2U glTextureSize() const override;
    Vec2U glTextureCoordinateOffset() const override;

    void incrementBufferUseCount();
    void decrementBufferUseCount();

    unsigned bufferUseCount = 0;
    shared_ptr<GlTextureGroup> parentGroup;
    GlTextureAtlasSet::TextureHandle parentAtlasTexture = nullptr;
  };

  struct GlLoneTexture : public GlTexture {
    ~GlLoneTexture();

    Vec2U size() const override;
    TextureFiltering filtering() const override;
    TextureAddressing addressing() const override;

    GLuint glTextureId() const override;
    Vec2U glTextureSize() const override;
    Vec2U glTextureCoordinateOffset() const override;

    GLuint textureId = 0;
    Vec2U textureSize;
    TextureAddressing textureAddressing = TextureAddressing::Clamp;
    TextureFiltering textureFiltering = TextureFiltering::Nearest;
  };

  struct GlPackedVertexData {
    uint32_t textureIndex : 2;
    uint32_t fullbright : 1;
    uint32_t rX : 1;
    uint32_t rY : 1;
    uint32_t unused : 27;
  };

  struct GlRenderVertex {
    Vec2F pos;
    Vec2F uv;
    Vec4B color;
    union Packed {
      uint32_t packed;
      GlPackedVertexData vars;
    } pack;
  };

  struct GlRenderBuffer : public RenderBuffer {
    struct GlVertexBufferTexture {
      GLuint texture;
      Vec2U size;
    };

    struct GlVertexBuffer {
      List<GlVertexBufferTexture> textures;
      GLuint vertexBuffer = 0;
      size_t vertexCount = 0;
    };

    GlRenderBuffer();
    ~GlRenderBuffer();

    void set(List<RenderPrimitive>& primitives) override;

    RefPtr<GlTexture> whiteTexture;
    ByteArray accumulationBuffer;

    HashSet<TexturePtr> usedTextures;
    List<GlVertexBuffer> vertexBuffers;
    GLuint vertexArray = 0;

    bool useMultiTexturing{true};
  };

  struct EffectParameter {
    GLint parameterUniform = -1;
    VariantTypeIndex parameterType = 0;
    Maybe<RenderEffectParameter> parameterValue;
  };

  struct EffectTexture {
    GLint textureUniform = -1;
    unsigned textureUnit = 0;
    TextureAddressing textureAddressing = TextureAddressing::Clamp;
    TextureFiltering textureFiltering = TextureFiltering::Linear;
    GLint textureSizeUniform = -1;
    RefPtr<GlLoneTexture> textureValue;
  };
  
  struct GlFrameBuffer : RefCounter {
    GLuint id = 0;
    RefPtr<GlLoneTexture> texture;

    Json config;
    bool blitted = false;
    unsigned multisample = 0;
    unsigned sizeDiv = 1;

    GlFrameBuffer(Json const& config);
    ~GlFrameBuffer();
  };

  class Effect {
  public:
    GLuint program = 0;
    Json config;
    StringMap<EffectParameter> parameters;
    StringMap<EffectParameter> scriptables; // scriptable parameters which can be changed when the effect is not loaded
    StringMap<EffectTexture> textures;

    StringMap<GLuint> attributes;
    StringMap<GLuint> uniforms;

    GLuint getAttribute(String const& name);
    GLuint getUniform(String const& name);
    bool includeVBTextures;
  };

  static bool logGlErrorSummary(String prefix);
  static void uploadTextureImage(PixelFormat pixelFormat, Vec2U size, uint8_t const* data);

  
  static RefPtr<GlLoneTexture> createGlTexture(ImageView const& image, TextureAddressing addressing, TextureFiltering filtering);

  shared_ptr<GlRenderBuffer> createGlRenderBuffer();

  void flushImmediatePrimitives(Mat3F const& transformation = Mat3F::identity());

  void renderGlBuffer(GlRenderBuffer const& renderBuffer, Mat3F const& transformation);

  void setupGlUniforms(Effect& effect, Vec2U screenSize);

  RefPtr<OpenGlRenderer::GlFrameBuffer> getGlFrameBuffer(String const& id);
  void blitGlFrameBuffer(RefPtr<OpenGlRenderer::GlFrameBuffer> const& frameBuffer);
  void switchGlFrameBuffer(RefPtr<OpenGlRenderer::GlFrameBuffer> const& frameBuffer);

  Vec2U m_screenSize;

  GLuint m_program = 0;

  GLint m_positionAttribute = -1;
  GLint m_colorAttribute = -1;
  GLint m_texCoordAttribute = -1;
  GLint m_dataAttribute = -1;
  List<GLint> m_textureUniforms = {};
  List<GLint> m_textureSizeUniforms = {};
  GLint m_screenSizeUniform = -1;
  GLint m_vertexTransformUniform = -1;

  Json m_config;

  StringMap<Effect> m_effects;
  Effect* m_currentEffect;

  StringMap<RefPtr<GlFrameBuffer>> m_frameBuffers;
  RefPtr<GlFrameBuffer> m_currentFrameBuffer;

  RefPtr<GlTexture> m_whiteTexture;

  Maybe<RectI> m_scissorRect;

  bool m_limitTextureGroupSize;
  bool m_useMultiTexturing;
  unsigned m_multiSampling; // if non-zero, is enabled and acts as sample count
  List<shared_ptr<GlTextureGroup>> m_liveTextureGroups;

  List<RenderPrimitive> m_immediatePrimitives;
  shared_ptr<GlRenderBuffer> m_immediateRenderBuffer;
};

namespace V2 {

STAR_CLASS(OpenGlRenderer);

class GlMappedBuffer : public V2::MappedBuffer {
public:
  GlMappedBuffer(uint32_t size);
  virtual ~GlMappedBuffer() noexcept override;

  void setFence() override;
  void waitFence() override;
  void upload(void const* data, uint32_t size, uint32_t offset) override;
  uint32_t handle() override;
  uint32_t size() override;

private:
  GLuint m_bufferHandle;
  GLsync m_fence = nullptr;
  void* m_map = nullptr;
  uint32_t m_size = 0;
};

class GlArenaBuffer : public V2::ArenaBuffer {
public:
  GlArenaBuffer(uint32_t size);
  ~GlArenaBuffer() noexcept override = default;

  BufferView allocateAlignedStorage(uint32_t size, uint32_t alignment) override;
  void setFence() override;
  void waitFence() override;

private:
  MappedBufferPtr m_buffer;
  uint32_t m_end = 0;
};

// OpenGL 4.6 implementation of the renderer for Windows and Linux.
// Dispatches calls to the legacy renderer on MacOS
class OpenGlRenderer : public Star::OpenGlRenderer, public V2::Renderer {
public:
  class GlBindlessTexture : public Star::OpenGlRenderer::GlTexture, public PooledTexture {
  public:
    virtual ~GlBindlessTexture();

    Vec2U size() const override;
    TextureFiltering filtering() const override;
    TextureAddressing addressing() const override;

    GLuint glTextureId() const;
    Vec2U glTextureSize() const;
    Vec2U glTextureCoordinateOffset() const override; // unused but required to interop with the old renderer
    uint64_t handle() const override;
    uint32_t poolIndex() const override;

    GLuint textureId = 0;
    Vec2U textureSize;
    TextureAddressing textureAddressing = TextureAddressing::Clamp;
    TextureFiltering textureFiltering = TextureFiltering::Nearest;

    GLuint64 residentHandle = 0;
    uint32_t m_poolIndex = 0;
  };

  OpenGlRenderer();
  virtual ~OpenGlRenderer() = default;

  void submit(CommandBuffer const& cmd) override;

  bool v2Available();

  virtual TexturePtr createTexture(Image const& texture, TextureAddressing addressing, TextureFiltering filtering) override;

  // Retrieves a pooled texture handle or loads it from the image path if it's unloaded
  PooledTexturePtr loadPooledTexture(AssetPath const& imagePath) override;

  MappedBufferPtr unitQuad() override;
  ArenaBuffer& shaderStorage() override;
  MappedBufferPtr instanceData();

  MappedBufferPtr texturePool();

  void startFrame() override;
  void finishFrame() override;

private:
    uint32_t translateBarrierBits(MemoryBarrierBits bits) override;

    bool m_v2Available = true;
    uint32_t m_frameIndex = 0;
    std::array<GLsync, 3> m_frameFences = { nullptr, nullptr, nullptr };

    const size_t m_maxTextures = 50000; // 400kb vram (texture handle is 8 bytes)
    const size_t m_shaderStorageSize = 2.5e7; // 25mb vram

    GLuint m_emptyVao; // OpenGL requires that a VAO is bound for a draw call but we're not using it (actually might not be necessary idk 100%)
    GlArenaBuffer m_shaderStorage;
    MappedBufferPtr m_texturePool;
    MappedBufferPtr m_instanceData;
    size_t m_poolEndOffset = 0; // offset in bytes to the next available slot in the texture pool
    std::unordered_map<ImageConstPtr, PooledTexturePtr> m_textureMap;

    struct ConfigKey {
      PipelineType type;
      std::string configName;
      bool operator==(const ConfigKey& other) const {
        return type == other.type && configName == other.configName;
      }
    };

    struct ConfigKeyHash {
      std::size_t operator()(const ConfigKey& k) const {
          return std::hash<PipelineType>{}(k.type) ^ (std::hash<std::string>{}(k.configName) << 1);
      }
    };

    std::unordered_map<ConfigKey, GLuint, ConfigKeyHash> m_programConfigMap;

    MappedBufferPtr m_unitQuad;

    GLuint getProgramConfig(PipelineType type, String const& programConfig);
    RefPtr<GlBindlessTexture> createGlBindlessTexture(ImageView const& texture, TextureAddressing addressing, TextureFiltering filtering);
};

}

}
