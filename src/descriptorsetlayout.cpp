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

#include "descriptorsetlayout.hpp"
#include "device.hpp"

DescriptorSetLayout::DescriptorSetLayout(Device const &a_device): 
  m_vulkan_descriptor_set_layout()
{
  auto vulkan_device = a_device.GetVulkanDevice();
  
  CreateVulkanDescriptorSetLayout(*vulkan_device);
}

DescriptorSetLayout::~DescriptorSetLayout()
{
}

int8_t DescriptorSetLayout::CreateVulkanDescriptorSetLayout(
    VkDevice const &a_vulkan_device)
{
  VkDescriptorSetLayoutBinding ubo_layout_binding = {};
  ubo_layout_binding.binding = 0;
  ubo_layout_binding.descriptorCount = 1;
  ubo_layout_binding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
  ubo_layout_binding.pImmutableSamplers = nullptr;
  ubo_layout_binding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
        
  VkDescriptorSetLayoutBinding sampler_layout_binding = {};
  sampler_layout_binding.binding = 1;
  sampler_layout_binding.descriptorCount = 1;
  sampler_layout_binding.descriptorType = 
    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  sampler_layout_binding.pImmutableSamplers = nullptr;
  sampler_layout_binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

  std::array<VkDescriptorSetLayoutBinding, 2> bindings = {ubo_layout_binding,
    sampler_layout_binding};
  VkDescriptorSetLayoutCreateInfo create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  create_info.bindingCount = static_cast<uint32_t>(bindings.size());
  create_info.pBindings = bindings.data();

  m_vulkan_descriptor_set_layout = std::shared_ptr<VkDescriptorSetLayout>(
      new VkDescriptorSetLayout, 
      [a_vulkan_device](VkDescriptorSetLayout *a_vulkan_descriptor_set_layout) {
        vkDestroyDescriptorSetLayout(a_vulkan_device, 
            *a_vulkan_descriptor_set_layout, nullptr);
        std::cout << "DEBUG: Descriptor set layout deleted." << std::endl;
      });

  int32_t res = vkCreateDescriptorSetLayout(a_vulkan_device, &create_info, 
      nullptr, &(*m_vulkan_descriptor_set_layout));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create descriptor set layout." << std::endl;
    return -1;
  }

  return 0;
}
    
std::shared_ptr<VkDescriptorSetLayout> 
DescriptorSetLayout::GetVulkanDescriptorSetLayout() const
{
  return m_vulkan_descriptor_set_layout;
}
