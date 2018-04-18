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

#ifndef TEXTURERESOURCES_HPP
#define TEXTURERESOURCES_HPP

#include <memory>
#include <string>

#include <vulkan/vulkan.h>

class CommandPool;
class Device;

/**
 * @brief A wrapper for Vulkan texture resources.
 */
class TextureResources {
  public:
    TextureResources(Device const &, CommandPool const &, std::string const &);
    TextureResources(TextureResources const &) = delete;
    TextureResources &operator=(TextureResources const &) = delete;
    virtual ~TextureResources();
    std::shared_ptr<VkImageView> GetVulkanTextureImageView() const;
  
  private:
    int8_t CreateVulkanTextureImage(Device const &, CommandPool const &, 
        std::string const &);
    int8_t CreateVulkanTextureImageView(VkDevice const &);
    int8_t TransitionImageLayout(Device const &, CommandPool const &, 
        VkImageLayout, VkImageLayout, VkAccessFlagBits, VkAccessFlagBits,
        VkPipelineStageFlags, VkPipelineStageFlags);

    std::shared_ptr<VkDeviceMemory> m_vulkan_texture_image_memory;
    std::shared_ptr<VkImage> m_vulkan_texture_image;
    std::shared_ptr<VkImageView> m_vulkan_texture_image_view;
};

#endif
