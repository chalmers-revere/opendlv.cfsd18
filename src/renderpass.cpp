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

#include <fstream>
#include <sstream>

#include "device.hpp"
#include "renderpass.hpp"
#include "swapchain.hpp"

RenderPass::RenderPass(Device const &a_device, Swapchain const &a_swapchain): 
  m_vulkan_render_pass(),
  m_vulkan_depth_format()
{
  auto vulkan_device = a_device.GetVulkanDevice();
  auto vulkan_swapchain_image_format = 
    a_swapchain.GetVulkanSwapchainImageFormat();
  
  m_vulkan_depth_format = FindVulkanDepthFormat(a_device);
  
  CreateVulkanRenderPass(*vulkan_device, vulkan_swapchain_image_format);
}

RenderPass::~RenderPass()
{
}

int8_t RenderPass::CreateVulkanRenderPass(VkDevice const &a_vulkan_device,
    VkFormat a_vulkan_swapchain_image_format)
{
  VkAttachmentDescription color_attachment = {};
  color_attachment.format = a_vulkan_swapchain_image_format;
  color_attachment.samples = VK_SAMPLE_COUNT_1_BIT;
  color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  color_attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  color_attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  color_attachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  color_attachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

  VkAttachmentDescription depth_attachment = {};
  depth_attachment.format = m_vulkan_depth_format;
  depth_attachment.samples = VK_SAMPLE_COUNT_1_BIT;
  depth_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  depth_attachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  depth_attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  depth_attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  depth_attachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  depth_attachment.finalLayout = 
    VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

  VkAttachmentReference color_attachment_ref = {};
  color_attachment_ref.attachment = 0;
  color_attachment_ref.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        
  VkAttachmentReference depth_attachment_ref = {};
  depth_attachment_ref.attachment = 1;
  depth_attachment_ref.layout = 
    VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

  VkSubpassDescription subpass = {};
  subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
  subpass.colorAttachmentCount = 1;
  subpass.pColorAttachments = &color_attachment_ref;
  subpass.pDepthStencilAttachment = &depth_attachment_ref;

  VkSubpassDependency dependency = {};
  dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
  dependency.dstSubpass = 0;
  dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependency.srcAccessMask = 0;
  dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
    VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

  std::array<VkAttachmentDescription, 2> attachments = {color_attachment,
    depth_attachment};
  VkRenderPassCreateInfo create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
  create_info.attachmentCount = static_cast<uint32_t>(attachments.size());;
  create_info.pAttachments = attachments.data();
  create_info.subpassCount = 1;
  create_info.pSubpasses = &subpass;
  create_info.dependencyCount = 1;
  create_info.pDependencies = &dependency;

  m_vulkan_render_pass = std::shared_ptr<VkRenderPass>(new VkRenderPass, 
      [a_vulkan_device](VkRenderPass *a_vulkan_render_pass) {
        vkDestroyRenderPass(a_vulkan_device, *a_vulkan_render_pass, nullptr);
        std::cout << "DEBUG: Render pass deleted." << std::endl;
      });

  int32_t res = vkCreateRenderPass(a_vulkan_device, &create_info, nullptr, 
      &(*m_vulkan_render_pass));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create render pass." << std::endl;
    return -1;
  }

  return 0;
}

VkFormat RenderPass::FindVulkanDepthFormat(Device const &a_device)
{
  std::vector<VkFormat> candidates = {VK_FORMAT_D32_SFLOAT,
    VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT};

  VkImageTiling tiling = VK_IMAGE_TILING_OPTIMAL;
  VkFormatFeatureFlags features = 
    VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT;

  for (VkFormat candidate : candidates) {
    VkFormatProperties format_properties = 
      a_device.GetVulkanFormatProperties(candidate);

    if (tiling == VK_IMAGE_TILING_LINEAR && 
        (format_properties.linearTilingFeatures & features) == features) {
      return candidate;
    } else if (tiling == VK_IMAGE_TILING_OPTIMAL && 
        (format_properties.optimalTilingFeatures & features) == features) {
      return candidate;
    }
  }

  std::cerr << "Failed to find supported format." << std::endl;
  return VK_FORMAT_UNDEFINED;
}

VkFormat RenderPass::GetVulkanDepthFormat() const
{
  return m_vulkan_depth_format;
}

std::shared_ptr<VkRenderPass> RenderPass::GetVulkanRenderPass() const
{
  return m_vulkan_render_pass;
}
