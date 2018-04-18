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
#include <iostream>
#include <vector>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "commandbuffers.hpp"
#include "commandpool.hpp"
#include "device.hpp"
#include "depthresources.hpp"
#include "descriptorsetlayout.hpp"
#include "descriptorsets.hpp"
#include "framebuffers.hpp"
#include "graphicspipeline.hpp"
#include "imageviews.hpp"
#include "instance.hpp"
#include "mesh.hpp"
#include "meshbuffers.hpp"
#include "renderpass.hpp"
#include "semaphore.hpp"
#include "surface.hpp"
#include "swapchain.hpp"
#include "textureresources.hpp"
#include "texturesampler.hpp"
#include "uniformbuffer.hpp"
#include "uniformbufferobject.hpp"
#include "vulkan.hpp"

Vulkan::Vulkan(std::shared_ptr<GLFWwindow> a_window, 
    std::string const &a_application_name, uint32_t const a_width, 
        uint32_t const a_height, bool const a_enable_validation_layers):
  m_instance(),
  m_surface(),
  m_device(),
  m_swapchain(),
  m_image_views(),
  m_render_pass(),
  m_descriptor_set_layout(),
  m_graphics_pipeline(),
  m_command_pool(),
  m_depth_resources(),
  m_framebuffers(),
  m_texture_resources(),
  m_texture_sampler(),
  m_mesh(),
  m_mesh_buffers(),
  m_uniform_buffer(),
  m_descriptor_sets(),
  m_command_buffers(),
  m_image_available_semaphore(),
  m_render_finished_semaphore()
{
  m_instance.reset(new Instance(a_application_name, 
        a_enable_validation_layers));
  m_surface.reset(new Surface(*a_window, *m_instance));
  m_device.reset(new Device(*m_instance, *m_surface));
  m_swapchain.reset(new Swapchain(*m_surface, *m_device, a_width, a_height));
  m_image_views.reset(new ImageViews(*m_device, *m_swapchain));
  m_render_pass.reset(new RenderPass(*m_device, *m_swapchain));
  m_descriptor_set_layout.reset(new DescriptorSetLayout(*m_device));
  m_graphics_pipeline.reset(new GraphicsPipeline(*m_device, *m_swapchain,
        *m_render_pass, *m_descriptor_set_layout,
        "lib/opendlv-device-gpu-vulkan/shader_depth.vert.spv", 
        "lib/opendlv-device-gpu-vulkan/shader_depth.frag.spv"));
  m_command_pool.reset(new CommandPool(*m_device));
  m_depth_resources.reset(new DepthResources(*m_device, *m_swapchain, 
        *m_render_pass, *m_command_pool));
  m_framebuffers.reset(new Framebuffers(*m_device, *m_swapchain, *m_image_views,
        *m_render_pass, *m_depth_resources));
  m_texture_resources.reset(new TextureResources(*m_device, *m_command_pool,
        "share/opendlv-device-gpu-vulkan/chalet.jpg"));
  m_texture_sampler.reset(new TextureSampler(*m_device));
  m_mesh.reset(new Mesh("share/opendlv-device-gpu-vulkan/chalet.obj"));
  m_mesh_buffers.reset(new MeshBuffers(*m_device, *m_command_pool, *m_mesh));
  m_uniform_buffer.reset(new UniformBuffer(*m_device));
  m_descriptor_sets.reset(new DescriptorSets(*m_device, 
        *m_descriptor_set_layout, *m_uniform_buffer, *m_texture_resources, 
        *m_texture_sampler));
  m_command_buffers.reset(new CommandBuffers(*m_device, *m_swapchain, 
        *m_render_pass, *m_graphics_pipeline, *m_command_pool, *m_framebuffers,
        *m_mesh_buffers, *m_descriptor_sets));
  m_image_available_semaphore.reset(new Semaphore(*m_device));
  m_render_finished_semaphore.reset(new Semaphore(*m_device));
}

Vulkan::~Vulkan()
{
}

void Vulkan::UpdateUniformBufferObject(double const a_time)
{

  VkExtent2D vulkan_swapchain_extent = m_swapchain->GetVulkanSwapchainExtent();
  float const width = static_cast<float>(vulkan_swapchain_extent.width);
  float const height = static_cast<float>(vulkan_swapchain_extent.height);
  float const aspect_ratio = width / height;

  UniformBufferObject ubo = {};
  ubo.model = glm::rotate(glm::mat4(), 
      static_cast<float>(a_time) * glm::radians(45.0f),
      glm::vec3(0.0f, 0.0f, 1.0f));
  ubo.view = glm::lookAt(glm::vec3(2.0f, 2.0f, 2.0f), 
      glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
  ubo.projection = glm::perspective(glm::radians(45.0f), aspect_ratio, 0.1f, 
      10.0f);
  ubo.projection[1][1] *= -1;

  m_uniform_buffer->UpdateUniformBufferObject(*m_device, ubo);
}

int8_t Vulkan::DrawFrame(uint32_t const a_width, uint32_t const a_height)
{
  auto vulkan_device = m_device->GetVulkanDevice();
  auto vulkan_graphics_queue = m_device->GetVulkanGraphicsQueue();
  auto vulkan_present_queue = m_device->GetVulkanPresentQueue();
  auto vulkan_swapchain = m_swapchain->GetVulkanSwapchain();
  auto vulkan_command_buffers = m_command_buffers->GetVulkanCommandBuffers();
  auto vulkan_image_available_semaphore = 
    m_image_available_semaphore->GetVulkanSemaphore();
  auto vulkan_render_finished_semaphore = 
    m_render_finished_semaphore->GetVulkanSemaphore();

  uint32_t image_index;
  VkResult vulkan_result = vkAcquireNextImageKHR(*vulkan_device, 
      *vulkan_swapchain, std::numeric_limits<uint64_t>::max(),
      *vulkan_image_available_semaphore, VK_NULL_HANDLE, &image_index);

  if (vulkan_result == VK_ERROR_OUT_OF_DATE_KHR) {
      RecreateSwapchain(a_width, a_height);
      return 0;
  } else if (vulkan_result != VK_SUCCESS 
      && vulkan_result != VK_SUBOPTIMAL_KHR) {
    std::cerr << "Failed to get swapchain image." 
      << std::endl;
    return -1;
  }

  VkSubmitInfo submit_info = {};
  submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

  VkSemaphore wait_semaphores[] = {*vulkan_image_available_semaphore};
  VkPipelineStageFlags wait_stages[] =
    {VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};
  submit_info.waitSemaphoreCount = 1;
  submit_info.pWaitSemaphores = wait_semaphores;
  submit_info.pWaitDstStageMask = wait_stages;

  submit_info.commandBufferCount = 1;
  submit_info.pCommandBuffers = &(*vulkan_command_buffers)[image_index];

  VkSemaphore signal_semaphores[] = {*vulkan_render_finished_semaphore};
  submit_info.signalSemaphoreCount = 1;
  submit_info.pSignalSemaphores = signal_semaphores;

  uint32_t res = vkQueueSubmit(*vulkan_graphics_queue, 1, &submit_info,
      VK_NULL_HANDLE);
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to submit draw command buffer." << std::endl;
    return -1;
  }

  VkPresentInfoKHR present_info = {};
  present_info.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;

  present_info.waitSemaphoreCount = 1;
  present_info.pWaitSemaphores = signal_semaphores;

  VkSwapchainKHR swapchains[] = {*vulkan_swapchain};
  present_info.swapchainCount = 1;
  present_info.pSwapchains = swapchains;

  present_info.pImageIndices = &image_index;

  vulkan_result = vkQueuePresentKHR(*vulkan_present_queue, &present_info);

  if (vulkan_result == VK_ERROR_OUT_OF_DATE_KHR 
      || vulkan_result == VK_SUBOPTIMAL_KHR) {
    RecreateSwapchain(a_width, a_height);
    return 0;
  } else if (vulkan_result != VK_SUCCESS) {
    std::cerr << "Failed to present swapchain image." << std::endl;
    return -1;
  }

  vkQueueWaitIdle(*vulkan_present_queue);

  return 0;
}

void Vulkan::RecreateSwapchain(uint32_t const a_width, uint32_t const a_height)
{
  auto vulkan_device = m_device->GetVulkanDevice();

  vkDeviceWaitIdle(*vulkan_device);
 
  std::cout << "Recreating swapchain." << std::endl;

  m_command_buffers.reset();
  m_framebuffers.reset();
  m_depth_resources.reset();
  m_graphics_pipeline.reset();
  m_render_pass.reset();
  m_image_views.reset();
  m_swapchain.reset();

  m_swapchain.reset(new Swapchain(*m_surface, *m_device, a_width, a_height));
  m_image_views.reset(new ImageViews(*m_device, *m_swapchain));
  m_render_pass.reset(new RenderPass(*m_device, *m_swapchain));
  m_graphics_pipeline.reset(new GraphicsPipeline(*m_device, *m_swapchain,
        *m_render_pass, *m_descriptor_set_layout,
        "lib/opendlv-device-gpu-vulkan/shader_depth.vert.spv", 
        "lib/opendlv-device-gpu-vulkan/shader_depth.frag.spv"));
  m_depth_resources.reset(new DepthResources(*m_device, *m_swapchain, 
        *m_render_pass, *m_command_pool));
  m_framebuffers.reset(new Framebuffers(*m_device, *m_swapchain, *m_image_views,
        *m_render_pass, *m_depth_resources));
  m_command_buffers.reset(new CommandBuffers(*m_device, *m_swapchain, 
        *m_render_pass, *m_graphics_pipeline, *m_command_pool, *m_framebuffers,
        *m_mesh_buffers, *m_descriptor_sets));
}
