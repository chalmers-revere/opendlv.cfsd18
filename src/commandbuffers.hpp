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

#ifndef COMMANDBUFFERS_HPP
#define COMMANDBUFFERS_HPP

#include <memory>
#include <vector>

#include <vulkan/vulkan.h>

class CommandPool;
class DescriptorSets;
class Device;
class Framebuffers;
class GraphicsPipeline;
class MeshBuffers;
class RenderPass;
class Swapchain;

/**
 * @brief A wrapper for a Vulkan uniform buffer.
 */
class CommandBuffers {
  public:
    CommandBuffers(Device const &, Swapchain const &, RenderPass const &,
        GraphicsPipeline const &, CommandPool const &, Framebuffers const &,
        MeshBuffers const &, DescriptorSets const &);
    CommandBuffers(CommandBuffers const &) = delete;
    CommandBuffers &operator=(CommandBuffers const &) = delete;
    virtual ~CommandBuffers();
    std::shared_ptr<std::vector<VkCommandBuffer>> GetVulkanCommandBuffers() 
      const;
  
  private:
    int8_t CreateVulkanCommandBuffers(Framebuffers const &, VkDevice const &, 
        VkRenderPass const &, VkPipeline const &, VkPipelineLayout const &, 
        VkCommandPool const &, VkBuffer const &, VkBuffer const &, 
        VkDescriptorSet const &, VkExtent2D, uint32_t);

    std::shared_ptr<std::vector<VkCommandBuffer>> m_vulkan_command_buffers;
};

#endif
