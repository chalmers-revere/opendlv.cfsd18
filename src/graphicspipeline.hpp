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

#ifndef GRAPHICSPIPELINE_HPP
#define GRAPHICSPIPELINE_HPP

#include <string>

#include <vulkan/vulkan.h>

class Device;
class DescriptorSetLayout;
class RenderPass;
class Swapchain;

/**
 * @brief A wrapper for a Vulkan graphics pipeline.
 */
class GraphicsPipeline {
  public:
    GraphicsPipeline(Device const &, Swapchain const &, RenderPass const &, 
        DescriptorSetLayout const &, std::string const &, std::string const &);
    GraphicsPipeline(GraphicsPipeline const &) = delete;
    GraphicsPipeline &operator=(GraphicsPipeline const &) = delete;
    virtual ~GraphicsPipeline();
    std::shared_ptr<VkPipeline> GetVulkanPipeline() const;
    std::shared_ptr<VkPipelineLayout> GetVulkanPipelineLayout() const;
  
  private:
    int8_t CreateVulkanGraphicsPipeline(VkDevice const &, VkRenderPass const &, 
        VkDescriptorSetLayout const &, VkExtent2D);
    std::shared_ptr<VkShaderModule> CreateVulkanShader(VkDevice const &, 
        std::string const &);

    std::shared_ptr<VkPipeline> m_vulkan_graphics_pipeline;
    std::shared_ptr<VkPipelineLayout> m_vulkan_pipeline_layout;
    std::shared_ptr<VkShaderModule> m_vulkan_fragment_shader;
    std::shared_ptr<VkShaderModule> m_vulkan_vertex_shader;
};

#endif
