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

#ifndef VULKAN_HPP
#define VULKAN_HPP

#include <memory>
#include <string>

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

class CommandBuffers;
class CommandPool;
class Device;
class DepthResources;
class DescriptorSetLayout;
class DescriptorSets;
class Framebuffers;
class GraphicsPipeline;
class Instance;
class ImageViews;
class Mesh;
class MeshBuffers;
class RenderPass;
class Semaphore;
class Surface;
class Swapchain;
class TextureResources;
class TextureSampler;
class UniformBuffer;

/**
 * @brief Used for creating a Vulkan context for a GLFW window. TODO: remove
 * GLFW references.
 */
class Vulkan {
  public:
    Vulkan(std::shared_ptr<GLFWwindow>, std::string const &, uint32_t const,
        uint32_t const, bool const);
    Vulkan(Vulkan const &) = delete;
    Vulkan &operator=(Vulkan const &) = delete;
    virtual ~Vulkan();
    void UpdateUniformBufferObject(double const);
    int8_t DrawFrame(uint32_t const, uint32_t const);
    void RecreateSwapchain(uint32_t const, uint32_t const);

  private:
    std::shared_ptr<Instance> m_instance;
    std::shared_ptr<Surface> m_surface;
    std::shared_ptr<Device> m_device;
    std::shared_ptr<Swapchain> m_swapchain;
    std::shared_ptr<ImageViews> m_image_views;
    std::shared_ptr<RenderPass> m_render_pass;
    std::shared_ptr<DescriptorSetLayout> m_descriptor_set_layout;
    std::shared_ptr<GraphicsPipeline> m_graphics_pipeline;
    std::shared_ptr<CommandPool> m_command_pool;
    std::shared_ptr<DepthResources> m_depth_resources;
    std::shared_ptr<Framebuffers> m_framebuffers;
    std::shared_ptr<TextureResources> m_texture_resources;
    std::shared_ptr<TextureSampler> m_texture_sampler;
    std::shared_ptr<Mesh> m_mesh;
    std::shared_ptr<MeshBuffers> m_mesh_buffers;
    std::shared_ptr<UniformBuffer> m_uniform_buffer;
    std::shared_ptr<DescriptorSets> m_descriptor_sets;
    std::shared_ptr<CommandBuffers> m_command_buffers;
    std::shared_ptr<Semaphore> m_image_available_semaphore;
    std::shared_ptr<Semaphore> m_render_finished_semaphore;
};

#endif
