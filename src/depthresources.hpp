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

#ifndef DEPTHRESOURCES_HPP
#define DEPTHRESOURCES_HPP

#include <memory>

#include <vulkan/vulkan.h>

class CommandPool;
class Device;
class RenderPass;
class Swapchain;

/**
 * @brief A wrapper for a Vulkan depth resources.
 */
class DepthResources {
  public:
    DepthResources(Device const &, Swapchain const &, RenderPass const &, 
        CommandPool const &);
    DepthResources(DepthResources const &) = delete;
    DepthResources &operator=(DepthResources const &) = delete;
    virtual ~DepthResources();
    std::shared_ptr<VkImageView> GetVulkanDepthImageView() const;
  
  private:
    int8_t CreateVulkanDepthImage(VkDevice const &, VkExtent2D, VkFormat);
    int8_t CreateVulkanDepthImageMemory(Device const &);
    int8_t CreateVulkanDepthImageView(VkDevice const &, VkFormat);
    int8_t TransitionImageLayout(Device const &, CommandPool const &, VkFormat);

    std::shared_ptr<VkDeviceMemory> m_vulkan_depth_image_memory;
    std::shared_ptr<VkImage> m_vulkan_depth_image;
    std::shared_ptr<VkImageView> m_vulkan_depth_image_view;
};

#endif
