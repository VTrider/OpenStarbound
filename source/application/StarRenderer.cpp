#include "StarRenderer.hpp"

#include "StarAssetPath.hpp"
#include "StarFormat.hpp"
#include "StarRoot.hpp"

namespace Star {

EnumMap<TextureAddressing> const TextureAddressingNames{
  {TextureAddressing::Clamp, "Clamp"},
  {TextureAddressing::Wrap, "Wrap"}
};

EnumMap<TextureFiltering> const TextureFilteringNames{
  {TextureFiltering::Nearest, "Nearest"},
  {TextureFiltering::Linear, "Linear"}
};

RenderQuad::RenderQuad(Vec2F posA, Vec2F posB, Vec2F posC, Vec2F posD, Vec4B color, float param1) : texture() {
  a = { posA, { 0, 0 }, color, param1 };
  b = { posB, { 0, 0 }, color, param1 };
  c = { posC, { 0, 0 }, color, param1 };
  d = { posD, { 0, 0 }, color, param1 };
}

RenderQuad::RenderQuad(TexturePtr tex, Vec2F minPosition, float textureScale, Vec4B color, float param1) : texture(std::move(tex)) {
  Vec2F size = Vec2F(texture->size());
  a = { minPosition, { 0, 0 }, color, param1};
  b = { { (minPosition[0] + size[0] * textureScale), minPosition[1] }, { size[0], 0 }, color, param1 };
  c = { { (minPosition[0] + size[0] * textureScale), (minPosition[1] + size[1] * textureScale) }, size, color, param1 };
  d = { { minPosition[0], (minPosition[1] + size[1] * textureScale) }, { 0, size[1] }, color, param1 };
}

RenderQuad::RenderQuad(TexturePtr tex, RectF const& screenCoords, Vec4B color, float param1) : texture(std::move(tex)) {
  Vec2F size = Vec2F(texture->size());
  a = { screenCoords.min(), { 0, 0 }, color, param1 };
  b = { { screenCoords.xMax(), screenCoords.yMin(), }, { size[0], 0.f }, color, param1 };
  c = { screenCoords.max(), size, color, param1};
  d = { { screenCoords.xMin(), screenCoords.yMax(), }, { 0.f, size[1] }, color, param1 };
}

RenderQuad::RenderQuad(TexturePtr tex, Vec2F posA, Vec2F uvA, Vec2F posB, Vec2F uvB, Vec2F posC, Vec2F uvC, Vec2F posD, Vec2F uvD, Vec4B color, float param1) : texture(std::move(tex)) {
  a = { posA, uvA, color, param1 };
  b = { posB, uvB, color, param1 };
  c = { posC, uvC, color, param1 };
  d = { posD, uvD, color, param1 };
}

RenderQuad::RenderQuad(TexturePtr tex, RenderVertex vA, RenderVertex vB, RenderVertex vC, RenderVertex vD)
  : texture(std::move(tex)), a(std::move(vA)), b(std::move(vB)), c(std::move(vC)), d(std::move(vD)) {}

RenderQuad::RenderQuad(RectF const& rect, Vec4B color, float param1)
  : a{ rect.min(), {}, color, param1 }
  , b{ { rect.xMax(), rect.yMin()}, {}, color, param1 }
  , c{ rect.max(), {}, color, param1 }
  , d{ { rect.xMin() ,rect.yMax() }, {}, color, param1 } {};


RenderPoly::RenderPoly(List<Vec2F> const& verts, Vec4B color, float param1) {
  vertexes.reserve(verts.size());
  for (Vec2F const& v : verts)
    vertexes.append({ v, { 0, 0 }, color, param1 });
}

RenderTriangle::RenderTriangle(Vec2F posA, Vec2F posB, Vec2F posC, Vec4B color, float param1) : texture() {
  a = { posA, { 0, 0 }, color, param1 };
  b = { posB, { 0, 0 }, color, param1 };
  c = { posC, { 0, 0 }, color, param1 };
}

RenderTriangle::RenderTriangle(TexturePtr tex, Vec2F posA, Vec2F uvA, Vec2F posB, Vec2F uvB, Vec2F posC, Vec2F uvC, Vec4B color, float param1) : texture(std::move(tex)) {
  a = { posA, uvA, color, param1 };
  b = { posB, uvB, color, param1 };
  c = { posC, uvC, color, param1 };
}

RenderQuad renderTexturedRect(TexturePtr texture, Vec2F minPosition, float textureScale, Vec4B color, float param1) {
  return RenderQuad(std::move(texture), minPosition, textureScale, color, param1);
}

RenderQuad renderTexturedRect(TexturePtr texture, RectF const& screenCoords, Vec4B color, float param1) {
  return RenderQuad(std::move(texture), screenCoords, color, param1);
}

RenderQuad renderFlatRect(RectF const& rect, Vec4B color, float param1) {
  return RenderQuad(rect, color, param1);
}

RenderPoly renderFlatPoly(PolyF const& poly, Vec4B color, float param1) {
  return RenderPoly(poly.vertexes(), color, param1);
}

namespace V2 {

BufferView::BufferView(MappedBufferPtr buf, uint32_t offset, uint32_t size) : m_buffer(buf), m_offset(offset), m_size(size) {
}

void BufferView::upload(void const* data, uint32_t size, uint32_t offset) {
  m_buffer->upload(data, size, m_offset + offset);
}

MappedBufferPtr BufferView::buffer() {
  return m_buffer;
}

uint32_t BufferView::offset() {
  return m_offset;
}

PipelineDescriptor& PipelineDescriptor::setType(PipelineType type) {
  m_type = type;
  return *this;
}

PipelineDescriptor& PipelineDescriptor::setProgram(String const& programConfig) {
  m_programConfig = programConfig;
  return *this;
}

DescriptorSet& DescriptorSet::bindUniformBuffer(uint32_t binding, MappedBufferPtr buf) {
  m_uniformBindings.emplace_back(binding, buf);
  return *this;
}

DescriptorSet& DescriptorSet::bindStorageBuffer(uint32_t binding, MappedBufferPtr buf) {
  if (binding == 0)
    throw RendererException("DescriptorSet::bindStorageBuffer: binding 0 is reserved for vertex attributes");

  m_storageBindings.emplace_back(binding, buf);
  return *this;
}

DescriptorSet& DescriptorSet::bindStorageBuffer(uint32_t binding, ArenaBuffer& buf) {
  return bindStorageBuffer(binding, buf.buffer());
}

CommandBuffer::CommandBuffer() {
  m_commandList.reserve(10);
}

CommandBuffer& CommandBuffer::bindVertexBuffer(MappedBufferPtr buffer) {
  List<CmdArg> args;
  args.emplace_back(buffer);
  m_commandList.emplace_back(CmdType::BindVertexBuffer, std::move(args));
  return *this;
}

CommandBuffer& CommandBuffer::bindPipeline(PipelineDescriptor const& pipeline) {
  List<CmdArg> args;
  args.emplace_back(&pipeline);
  m_commandList.emplace_back(CmdType::BindPipeline, std::move(args));
  return *this;
}

CommandBuffer& CommandBuffer::bindDescriptorSet(DescriptorSet const& descriptor) {
  List<CmdArg> args;
  args.emplace_back(&descriptor);
  m_commandList.emplace_back(CmdType::BindDescriptorSet, std::move(args));
  return *this;
}

CommandBuffer& CommandBuffer::pushConstant(uint32_t location, ProgramConstantType constant) {
  List<CmdArg> args;
  args.emplace_back(std::in_place_type_t<ProgramConstantInfo>(), location, constant);
  m_commandList.emplace_back(CmdType::PushConstant, std::move(args));
  return *this;
}

CommandBuffer& CommandBuffer::draw(uint32_t count, uint32_t instanceCount, uint32_t firstVertex, uint32_t firstInstance) {
  List<CmdArg> args;
  args.emplace_back(count);
  args.emplace_back(instanceCount);
  args.emplace_back(firstVertex);
  args.emplace_back(firstInstance);
  m_commandList.emplace_back(CmdType::Draw, std::move(args));
  return *this;
}

CommandBuffer& CommandBuffer::drawIndirect(MappedBufferPtr cmdBuffer, uint32_t offset, uint32_t drawCount, uint32_t stride) {
  List<CmdArg> args;
  args.emplace_back(cmdBuffer);
  args.emplace_back(offset);
  args.emplace_back(drawCount);
  args.emplace_back(stride);
  m_commandList.emplace_back(CmdType::Draw, std::move(args));
  return *this;
}

CommandBuffer& CommandBuffer::setFence(MappedBufferPtr buffer) {
  List<CmdArg> args;
  args.emplace_back(buffer);
  m_commandList.emplace_back(CmdType::SetFence, std::move(args));
  return *this;
}

CommandBuffer& CommandBuffer::dispatch(uint32_t groupCountX, uint32_t groupCountY, uint32_t groupCountZ) {
  List<CmdArg> args;
  args.emplace_back(groupCountX);
  args.emplace_back(groupCountY);
  args.emplace_back(groupCountZ);
  m_commandList.emplace_back(CmdType::Dispatch, std::move(args));
  return *this;
}

CommandBuffer& CommandBuffer::memoryBarrier(MemoryBarrierBits bits) {
  List<CmdArg> args;
  args.emplace_back(bits);
  m_commandList.emplace_back(CmdType::MemoryBarrier, std::move(args));
  return *this;
}


}// namespace V2

} // namespace Star

