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

#include "depthresources.hpp"
#include "device.hpp"
#include "framebuffers.hpp"
#include "imageviews.hpp"
#include "renderpass.hpp"
#include "swapchain.hpp"

Framebuffers::Framebuffers(Device const &a_device, Swapchain const &a_swapchain, 
    ImageViews const &a_image_views, RenderPass const &a_render_pass,
    DepthResources const &a_depth_resources):
  m_vulkan_framebuffers()
{
  auto vulkan_device = a_device.GetVulkanDevice();
  auto vulkan_render_pass = a_render_pass.GetVulkanRenderPass();
  auto vulkan_depth_image_view = a_depth_resources.GetVulkanDepthImageView();
  auto vulkan_swapchain_extent = a_swapchain.GetVulkanSwapchainExtent();

  CreateVulkanFramebuffers(a_image_views, *vulkan_device, *vulkan_render_pass,
      *vulkan_depth_image_view, vulkan_swapchain_extent);
}

Framebuffers::~Framebuffers()
{
}

int8_t Framebuffers::CreateVulkanFramebuffers(ImageViews const &a_image_views, 
    VkDevice const &a_vulkan_device, VkRenderPass const &a_vulkan_render_pass,
    VkImageView const &a_vulkan_depth_image_view,
    VkExtent2D a_vulkan_swapchain_extent)
{
  uint32_t image_view_count = a_image_views.GetImageViewCount();

  m_vulkan_framebuffers.resize(image_view_count);

  for (uint32_t i = 0; i < image_view_count; i++) {

    auto vulkan_image_view = a_image_views.GetVulkanImageView(i);

    std::array<VkImageView, 2> attachments = {*vulkan_image_view,
      a_vulkan_depth_image_view};

    VkFramebufferCreateInfo create_info = {};
    create_info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    create_info.renderPass = a_vulkan_render_pass;
    create_info.attachmentCount = static_cast<uint32_t>(attachments.size());
    create_info.pAttachments = attachments.data();
    create_info.width = a_vulkan_swapchain_extent.width;
    create_info.height = a_vulkan_swapchain_extent.height;
    create_info.layers = 1;

    m_vulkan_framebuffers[i] = std::shared_ptr<VkFramebuffer>(new VkFramebuffer, 
        [a_vulkan_device, i](VkFramebuffer *a_vulkan_framebuffer) {
          vkDestroyFramebuffer(a_vulkan_device, *a_vulkan_framebuffer, nullptr);
          std::cout << "DEBUG: Framebuffer " << i << " deleted." << std::endl;
        });

    int32_t res = vkCreateFramebuffer(a_vulkan_device, &create_info, nullptr, 
        &(*m_vulkan_framebuffers[i]));
    if (res != VK_SUCCESS) {
      std::cerr << "Failed to create framebuffer " << i << "." << std::endl;
      return -1;
    }
  }

  return 0;
}

uint32_t Framebuffers::GetFramebufferCount() const
{
  return m_vulkan_framebuffers.size();
}

std::shared_ptr<VkFramebuffer> Framebuffers::GetVulkanFramebuffer(
    uint32_t a_index) const
{
  return m_vulkan_framebuffers[a_index];
}
