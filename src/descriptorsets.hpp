/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef DESCRIPTORSETS_HPP
#define DESCRIPTORSETS_HPP

#include <memory>

#include <vulkan/vulkan.h>

class DescriptorSetLayout;
class Device;
class TextureResources;
class TextureSampler;
class UniformBuffer;

/**
 * @brief A wrapper for Vulkan descriptor pool and sets.
 */
class DescriptorSets {
  public:
    DescriptorSets(Device const &, DescriptorSetLayout const &, 
        UniformBuffer const &, TextureResources const &, 
        TextureSampler const &);
    DescriptorSets(DescriptorSets const &) = delete;
    DescriptorSets &operator=(DescriptorSets const &) = delete;
    virtual ~DescriptorSets();
    std::shared_ptr<VkDescriptorSet> GetVulkanDescriptorSet() const;
  
  private:
    int8_t CreateVulkanDescriptorPool(VkDevice const &);
    int8_t UpdateVulkanDescriptorSet(VkDevice const &, 
        VkDescriptorSetLayout const &, VkBuffer const &,
        VkImageView const &, VkSampler const &);

    std::shared_ptr<VkDescriptorPool> m_vulkan_descriptor_pool;
    std::shared_ptr<VkDescriptorSet> m_vulkan_descriptor_set;
};

#endif
