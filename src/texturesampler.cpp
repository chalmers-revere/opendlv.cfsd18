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

#include "device.hpp"
#include "texturesampler.hpp"

TextureSampler::TextureSampler(Device const &a_device): 
  m_vulkan_texture_sampler()
{
  auto vulkan_device = a_device.GetVulkanDevice();
  CreateVulkanTextureSampler(*vulkan_device); 
}

TextureSampler::~TextureSampler()
{
}

int8_t TextureSampler::CreateVulkanTextureSampler(
    VkDevice const &a_vulkan_device)
{
  VkSamplerCreateInfo create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  create_info.magFilter = VK_FILTER_LINEAR;
  create_info.minFilter = VK_FILTER_LINEAR;
  create_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  create_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  create_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  create_info.anisotropyEnable = VK_TRUE;
  create_info.maxAnisotropy = 16;
  create_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
  create_info.unnormalizedCoordinates = VK_FALSE;
  create_info.compareEnable = VK_FALSE;
  create_info.compareOp = VK_COMPARE_OP_ALWAYS;
  create_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;

  m_vulkan_texture_sampler = std::shared_ptr<VkSampler>(new VkSampler, 
      [a_vulkan_device](VkSampler *a_vulkan_texture_sampler) {
        vkDestroySampler(a_vulkan_device, *a_vulkan_texture_sampler, nullptr);
        std::cout << "DEBUG: Texture sampler deleted." << std::endl;
      });

  int32_t res = vkCreateSampler(a_vulkan_device, &create_info, nullptr, 
      &(*m_vulkan_texture_sampler));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create texture sampler." << std::endl;
    return -1;
  }

  return 0;
}

std::shared_ptr<VkSampler> TextureSampler::GetVulkanTextureSampler() const
{
  return m_vulkan_texture_sampler;
}
