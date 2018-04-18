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

#ifndef RENDERPASS_HPP
#define RENDERPASS_HPP

#include <vulkan/vulkan.h>

class Device;
class Swapchain;

/**
 * @brief A wrapper for a Vulkan render pass.
 */
class RenderPass {
  public:
    RenderPass(Device const &, Swapchain const &);
    RenderPass(RenderPass const &) = delete;
    RenderPass &operator=(RenderPass const &) = delete;
    virtual ~RenderPass();
    std::shared_ptr<VkRenderPass> GetVulkanRenderPass() const;
    VkFormat GetVulkanDepthFormat() const;
  
  private:
    int8_t CreateVulkanRenderPass(VkDevice const &, VkFormat);
    VkFormat FindVulkanDepthFormat(Device const &);

    std::shared_ptr<VkRenderPass> m_vulkan_render_pass;
    VkFormat m_vulkan_depth_format;
};

#endif
