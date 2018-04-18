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

#include <cstring>

#include "commandbuffers.hpp"
#include "commandpool.hpp"
#include "descriptorsets.hpp"
#include "device.hpp"
#include "framebuffers.hpp"
#include "graphicspipeline.hpp"
#include "meshbuffers.hpp"
#include "renderpass.hpp"
#include "swapchain.hpp"

CommandBuffers::CommandBuffers(Device const &a_device, 
    Swapchain const &a_swapchain, RenderPass const &a_render_pass, 
    GraphicsPipeline const &a_graphics_pipeline,
    CommandPool const &a_command_pool, Framebuffers const &a_framebuffers,
    MeshBuffers const &a_mesh_buffers, 
    DescriptorSets const & a_descriptor_sets): 
  m_vulkan_command_buffers()
{
  auto vulkan_device = a_device.GetVulkanDevice();
  auto vulkan_render_pass = a_render_pass.GetVulkanRenderPass();
  auto vulkan_pipeline = a_graphics_pipeline.GetVulkanPipeline();
  auto vulkan_pipeline_layout = 
    a_graphics_pipeline.GetVulkanPipelineLayout();
  auto vulkan_command_pool = a_command_pool.GetVulkanCommandPool();
  auto vulkan_vertex_buffer = a_mesh_buffers.GetVulkanVertexBuffer();
  auto vulkan_index_buffer = a_mesh_buffers.GetVulkanIndexBuffer();
  auto vulkan_descriptor_set = a_descriptor_sets.GetVulkanDescriptorSet();

  VkExtent2D vulkan_swapchain_extent = 
    a_swapchain.GetVulkanSwapchainExtent();
  uint32_t const index_count = a_mesh_buffers.GetIndexCount();

  CommandBuffers::CreateVulkanCommandBuffers(a_framebuffers, *vulkan_device,
      *vulkan_render_pass, *vulkan_pipeline, *vulkan_pipeline_layout,
      *vulkan_command_pool, *vulkan_vertex_buffer, *vulkan_index_buffer,
      *vulkan_descriptor_set, vulkan_swapchain_extent, index_count);
}

CommandBuffers::~CommandBuffers()
{
}

int8_t CommandBuffers::CreateVulkanCommandBuffers(
    Framebuffers const &a_framebuffers, VkDevice const &a_vulkan_device, 
    VkRenderPass const &a_vulkan_render_pass, 
    VkPipeline const &a_vulkan_pipeline, 
    VkPipelineLayout const &a_vulkan_pipeline_layout,
    VkCommandPool const &a_vulkan_command_pool, 
    VkBuffer const &a_vulkan_vertex_buffer,
    VkBuffer const &a_vulkan_index_buffer,
    VkDescriptorSet const &a_vulkan_descriptor_set, 
    VkExtent2D a_vulkan_swapchain_extent,
    uint32_t a_index_count)
{
  uint32_t framebuffer_count = a_framebuffers.GetFramebufferCount();
  
  VkCommandBufferAllocateInfo allocate_info = {};
  allocate_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  allocate_info.commandPool = a_vulkan_command_pool;
  allocate_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  allocate_info.commandBufferCount = framebuffer_count;
  
  m_vulkan_command_buffers = std::shared_ptr<std::vector<VkCommandBuffer>>(
      new std::vector<VkCommandBuffer>, 
      [a_vulkan_device, a_vulkan_command_pool](
          std::vector<VkCommandBuffer> *a_vulkan_command_buffers) {
        uint32_t n = static_cast<uint32_t>(a_vulkan_command_buffers->size());
        vkFreeCommandBuffers(a_vulkan_device, a_vulkan_command_pool, n,
            a_vulkan_command_buffers->data());
        std::cout << "DEBUG: Command buffers deleted." << std::endl;
      });
  
  m_vulkan_command_buffers->resize(framebuffer_count);
  
  int32_t res = vkAllocateCommandBuffers(a_vulkan_device, &allocate_info,
      m_vulkan_command_buffers->data());
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create command buffers." << std::endl;
    return -1;
  }
  
  for (size_t i = 0; i < framebuffer_count; i++) {
    VkCommandBuffer command_buffer = m_vulkan_command_buffers->at(i);
  
    VkCommandBufferBeginInfo begin_info = {};
    begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin_info.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;

    vkBeginCommandBuffer(command_buffer, &begin_info);
    
    VkRenderPassBeginInfo render_pass_info = {};
    render_pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    render_pass_info.renderPass = a_vulkan_render_pass;
    render_pass_info.framebuffer = *(a_framebuffers.GetVulkanFramebuffer(i));
    render_pass_info.renderArea.offset = {0, 0};
    render_pass_info.renderArea.extent = a_vulkan_swapchain_extent;
    
    std::array<VkClearValue, 2> clear_values = {};
    clear_values[0].color = {0.0f, 0.0f, 0.0f, 1.0f};
    clear_values[1].depthStencil = {1.0f, 0};

    render_pass_info.clearValueCount = 
      static_cast<uint32_t>(clear_values.size());
    render_pass_info.pClearValues = clear_values.data();

    vkCmdBeginRenderPass(command_buffer, &render_pass_info,
        VK_SUBPASS_CONTENTS_INLINE);

    vkCmdBindPipeline(command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, 
        a_vulkan_pipeline);
    
    VkBuffer vertex_buffers[] = {a_vulkan_vertex_buffer};
    VkDeviceSize offsets[] = {0};
    vkCmdBindVertexBuffers(command_buffer, 0, 1, vertex_buffers, offsets);

    vkCmdBindIndexBuffer(command_buffer, a_vulkan_index_buffer, 0,
        VK_INDEX_TYPE_UINT32);

    vkCmdBindDescriptorSets(command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, 
        a_vulkan_pipeline_layout, 0, 1, &a_vulkan_descriptor_set, 0, 
        nullptr);
    
    vkCmdDrawIndexed(command_buffer, a_index_count, 1, 0, 0, 0);

    vkCmdEndRenderPass(command_buffer);

    res = vkEndCommandBuffer(command_buffer);
    
    if (res != VK_SUCCESS) {
      std::cerr << "Failed to record command buffer for framebuffer " 
        << i << "." << std::endl;
      return -1;
    }
  }
  
  return 0;
}

std::shared_ptr<std::vector<VkCommandBuffer>> 
  CommandBuffers::GetVulkanCommandBuffers() const
{
  return m_vulkan_command_buffers;
}
