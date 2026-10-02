#pragma once
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/ogre2/Ogre2Scene.hh>
#include <OgreRoot.h>
#include <Compositor/OgreCompositorManager2.h>
#include <Compositor/OgreCompositorShadowNodeDef.h>

namespace ssb {
// Ogre's large default normal offset displaces receivers outside these narrow
// spotlight shadow frusta. Keep the existing maps and constant depth bias;
// adjust only the normal offset, on the rendering thread.
inline bool ApplyStripShadowSettings(float normal_offset = 1.f) {
  auto scene = std::dynamic_pointer_cast<gz::rendering::Ogre2Scene>(
      gz::rendering::sceneFromFirstRenderEngine());
  auto* root = Ogre::Root::getSingletonPtr();
  if (!scene || !root) return false;
  auto* manager = root->getCompositorManager2();
  const Ogre::IdString name("PbsMaterialsShadowNode");
  if (!manager->hasShadowNodeDefinition(name)) return false;
  auto* definition = manager->getShadowNodeDefinitionNonConst(name);
  bool changed = false;
  for (size_t i = 0; i < definition->getNumShadowTextureDefinitions(); ++i) {
    auto* map = definition->getShadowTextureDefinitionNonConst(i);
    if (map->normalOffsetBias != normal_offset) {
      map->normalOffsetBias = normal_offset;
      changed = true;
    }
  }
  return changed;
}
}  // namespace ssb
