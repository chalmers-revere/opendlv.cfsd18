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

#ifndef FRAMEBUFFERS_HPP
#define FRAMEBUFFERS_HPP

#include <memory>

#include <vulkan/vulkan.h>

class DepthResources;
class Device;
class ImageViews;
class RenderPass;
class Swapchain;

/**
 * @brief A wrapper for a set of Vulkan framebuffers.
 */
class Framebuffers {
  public:
    Framebuffers(Device const &, Swapchain const &, ImageViews const &,
        RenderPass const &, DepthResources const &);
    Framebuffers(Framebuffers const &) = delete;
    Framebuffers &operator=(Framebuffers const &) = delete;
    virtual ~Framebuffers();
    uint32_t GetFramebufferCount() const;
    std::shared_ptr<VkFramebuffer> GetVulkanFramebuffer(uint32_t) const;
  
  private:
    int8_t CreateVulkanFramebuffers(ImageViews const &, VkDevice const &,
        VkRenderPass const &, VkImageView const &, VkExtent2D);

    std::vector<std::shared_ptr<VkFramebuffer>> m_vulkan_framebuffers;
};

#endif
