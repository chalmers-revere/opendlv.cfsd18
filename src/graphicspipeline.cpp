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

#include "descriptorsetlayout.hpp"
#include "device.hpp"
#include "graphicspipeline.hpp"
#include "renderpass.hpp"
#include "swapchain.hpp"
#include "vertex.hpp"

GraphicsPipeline::GraphicsPipeline(Device const &a_device, 
    Swapchain const &a_swapchain, RenderPass const &a_render_pass,
    DescriptorSetLayout const &a_descriptor_set_layout,
    std::string const &a_vertex_shader_path, 
    std::string const &a_fragment_shader_path): 
  m_vulkan_graphics_pipeline(),
  m_vulkan_pipeline_layout(),
  m_vulkan_fragment_shader(),
  m_vulkan_vertex_shader()
{
  auto vulkan_device = a_device.GetVulkanDevice();
  auto vulkan_render_pass = a_render_pass.GetVulkanRenderPass();
  auto vulkan_descriptor_set_layout = 
    a_descriptor_set_layout.GetVulkanDescriptorSetLayout();
  auto vulkan_swapchain_extent = 
    a_swapchain.GetVulkanSwapchainExtent();
  
  m_vulkan_vertex_shader = CreateVulkanShader(*vulkan_device,
      a_vertex_shader_path);
  m_vulkan_fragment_shader = CreateVulkanShader(*vulkan_device,
      a_fragment_shader_path);

  CreateVulkanGraphicsPipeline(*vulkan_device, *vulkan_render_pass,
      *vulkan_descriptor_set_layout, vulkan_swapchain_extent);
}

GraphicsPipeline::~GraphicsPipeline()
{
}

int8_t GraphicsPipeline::CreateVulkanGraphicsPipeline(
    VkDevice const &a_vulkan_device, VkRenderPass const &a_vulkan_render_pass,
    VkDescriptorSetLayout const &a_vulkan_descriptor_set_layout,
    VkExtent2D a_vulkan_swapchain_extent)
{
  VkPipelineShaderStageCreateInfo vertex_shader_stage_create_info = {};
  vertex_shader_stage_create_info.sType = 
    VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  vertex_shader_stage_create_info.stage = VK_SHADER_STAGE_VERTEX_BIT;
  vertex_shader_stage_create_info.module = *m_vulkan_vertex_shader;
  vertex_shader_stage_create_info.pName = "main";

  VkPipelineShaderStageCreateInfo fragment_shader_stage_create_info = {};
  fragment_shader_stage_create_info.sType =
    VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  fragment_shader_stage_create_info.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
  fragment_shader_stage_create_info.module = *m_vulkan_fragment_shader;
  fragment_shader_stage_create_info.pName = "main";

  VkPipelineShaderStageCreateInfo shader_stage_create_info[] =
    {vertex_shader_stage_create_info, fragment_shader_stage_create_info};

  VkPipelineVertexInputStateCreateInfo vertex_input_create_info = {};
  vertex_input_create_info.sType =
    VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

  VkVertexInputBindingDescription binding_description = {};
  binding_description.binding = 0;
  binding_description.stride = sizeof(Vertex);
  binding_description.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

  std::array<VkVertexInputAttributeDescription, 3> attribute_descriptions = {};

  attribute_descriptions[0].binding = 0;
  attribute_descriptions[0].location = 0;
  attribute_descriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
  attribute_descriptions[0].offset = offsetof(Vertex, Vertex::position);

  attribute_descriptions[1].binding = 0;
  attribute_descriptions[1].location = 1;
  attribute_descriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
  attribute_descriptions[1].offset = offsetof(Vertex, Vertex::color);
  
  attribute_descriptions[2].binding = 0;
  attribute_descriptions[2].location = 2;
  attribute_descriptions[2].format = VK_FORMAT_R32G32_SFLOAT;
  attribute_descriptions[2].offset = offsetof(Vertex, Vertex::texture);

  vertex_input_create_info.vertexBindingDescriptionCount = 1;
  vertex_input_create_info.vertexAttributeDescriptionCount =
    static_cast<uint32_t>(attribute_descriptions.size());
  vertex_input_create_info.pVertexBindingDescriptions = &binding_description;
  vertex_input_create_info.pVertexAttributeDescriptions = 
    attribute_descriptions.data();

  VkPipelineInputAssemblyStateCreateInfo input_assembly = {};
  input_assembly.sType = 
    VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
  input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
  input_assembly.primitiveRestartEnable = VK_FALSE;

  VkViewport viewport = {};
  viewport.x = 0.0f;
  viewport.y = 0.0f;
  viewport.width = static_cast<float>(a_vulkan_swapchain_extent.width);
  viewport.height = static_cast<float>(a_vulkan_swapchain_extent.height);
  viewport.minDepth = 0.0f;
  viewport.maxDepth = 1.0f;

  VkRect2D scissor = {};
  scissor.offset = {0, 0};
  scissor.extent = a_vulkan_swapchain_extent;

  VkPipelineViewportStateCreateInfo viewport_state_create_info = {};
  viewport_state_create_info.sType =
    VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
  viewport_state_create_info.viewportCount = 1;
  viewport_state_create_info.pViewports = &viewport;
  viewport_state_create_info.scissorCount = 1;
  viewport_state_create_info.pScissors = &scissor;

  VkPipelineRasterizationStateCreateInfo rasterization_state_create_info = {};
  rasterization_state_create_info.sType =
    VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
  rasterization_state_create_info.depthClampEnable = VK_FALSE;
  rasterization_state_create_info.rasterizerDiscardEnable = VK_FALSE;
  rasterization_state_create_info.polygonMode = VK_POLYGON_MODE_FILL;
  rasterization_state_create_info.lineWidth = 1.0f;
  rasterization_state_create_info.cullMode = VK_CULL_MODE_BACK_BIT;
  rasterization_state_create_info.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
  rasterization_state_create_info.depthBiasEnable = VK_FALSE;

  VkPipelineMultisampleStateCreateInfo multisample_state_create_info = {};
  multisample_state_create_info.sType =
    VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
  multisample_state_create_info.sampleShadingEnable = VK_FALSE;
  multisample_state_create_info.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

  VkPipelineDepthStencilStateCreateInfo depth_stencil_state_create_info = {};
  depth_stencil_state_create_info.sType =
    VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
  depth_stencil_state_create_info.depthTestEnable = VK_TRUE;
  depth_stencil_state_create_info.depthWriteEnable = VK_TRUE;
  depth_stencil_state_create_info.depthCompareOp = VK_COMPARE_OP_LESS;
  depth_stencil_state_create_info.depthBoundsTestEnable = VK_FALSE;
  depth_stencil_state_create_info.stencilTestEnable = VK_FALSE;

  VkPipelineColorBlendAttachmentState color_blend_attachment_state = {};
  color_blend_attachment_state.colorWriteMask = VK_COLOR_COMPONENT_R_BIT |
    VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | 
    VK_COLOR_COMPONENT_A_BIT;
  color_blend_attachment_state.blendEnable = VK_FALSE;

  VkPipelineColorBlendStateCreateInfo color_blend_state_create_info = {};
  color_blend_state_create_info.sType =
    VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
  color_blend_state_create_info.logicOpEnable = VK_FALSE;
  color_blend_state_create_info.logicOp = VK_LOGIC_OP_COPY;
  color_blend_state_create_info.attachmentCount = 1;
  color_blend_state_create_info.pAttachments = &color_blend_attachment_state;
  color_blend_state_create_info.blendConstants[0] = 0.0f;
  color_blend_state_create_info.blendConstants[1] = 0.0f;
  color_blend_state_create_info.blendConstants[2] = 0.0f;
  color_blend_state_create_info.blendConstants[3] = 0.0f;

  VkPipelineLayoutCreateInfo pipeline_layout_create_info = {};
  pipeline_layout_create_info.sType = 
    VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  pipeline_layout_create_info.setLayoutCount = 1;
  pipeline_layout_create_info.pSetLayouts = &a_vulkan_descriptor_set_layout;

  m_vulkan_pipeline_layout = std::shared_ptr<VkPipelineLayout>(
      new VkPipelineLayout, 
      [a_vulkan_device](VkPipelineLayout *a_vulkan_pipeline_layout) {
        vkDestroyPipelineLayout(a_vulkan_device, *a_vulkan_pipeline_layout, 
            nullptr);
        std::cout << "DEBUG: Pipeline layout deleted." << std::endl;
      });

  int32_t res = vkCreatePipelineLayout(a_vulkan_device, 
      &pipeline_layout_create_info, nullptr, &(*m_vulkan_pipeline_layout));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create pipeline layout." << std::endl;
    return -1;
  }

  VkGraphicsPipelineCreateInfo graphics_pipeline_create_info = {};
  graphics_pipeline_create_info.sType =
    VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
  graphics_pipeline_create_info.stageCount = 2;
  graphics_pipeline_create_info.pStages = shader_stage_create_info;
  graphics_pipeline_create_info.pVertexInputState = &vertex_input_create_info;
  graphics_pipeline_create_info.pInputAssemblyState = &input_assembly;
  graphics_pipeline_create_info.pViewportState = &viewport_state_create_info;
  graphics_pipeline_create_info.pRasterizationState =
    &rasterization_state_create_info;
  graphics_pipeline_create_info.pMultisampleState = 
    &multisample_state_create_info;
  graphics_pipeline_create_info.pDepthStencilState =
    &depth_stencil_state_create_info;
  graphics_pipeline_create_info.pColorBlendState = 
    &color_blend_state_create_info;
  graphics_pipeline_create_info.layout = *m_vulkan_pipeline_layout;
  graphics_pipeline_create_info.renderPass = a_vulkan_render_pass;
  graphics_pipeline_create_info.subpass = 0;
  graphics_pipeline_create_info.basePipelineHandle = VK_NULL_HANDLE;

  m_vulkan_graphics_pipeline = std::shared_ptr<VkPipeline>(new VkPipeline, 
      [a_vulkan_device](VkPipeline *a_vulkan_pipeline) {
        vkDestroyPipeline(a_vulkan_device, *a_vulkan_pipeline, nullptr);
        std::cout << "DEBUG: Pipeline deleted." << std::endl;
      });

  res = vkCreateGraphicsPipelines(a_vulkan_device, VK_NULL_HANDLE, 1,
      &graphics_pipeline_create_info, nullptr, &(*m_vulkan_graphics_pipeline));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create graphics pipeline." << std::endl;
    return -1;
  }

  return 0;
}

std::shared_ptr<VkShaderModule> GraphicsPipeline::CreateVulkanShader(
    VkDevice const &a_vulkan_device, std::string const &a_shader_path)
{
  std::ifstream file(a_shader_path, std::ios::ate | std::ios::binary);

  if (!file.is_open()) {
    std::cerr << "Failed to open file '" << a_shader_path << "'." << std::endl;
    return nullptr;
  }

  size_t file_size = (size_t) file.tellg();
  std::vector<char> shader_source(file_size);

  file.seekg(0);
  file.read(shader_source.data(), file_size);

  file.close();


  VkShaderModuleCreateInfo create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  create_info.codeSize = shader_source.size();
  create_info.pCode = (uint32_t *) shader_source.data();

  std::shared_ptr<VkShaderModule> shader_module = 
    std::shared_ptr<VkShaderModule>(
      new VkShaderModule, 
      [a_vulkan_device, a_shader_path](VkShaderModule *a_vulkan_shader_module) {
        vkDestroyShaderModule(a_vulkan_device, *a_vulkan_shader_module, 
            nullptr);
        std::cout << "DEBUG: Shader module (from '" << a_shader_path << 
            "') deleted." << std::endl;
      });

  int32_t res = vkCreateShaderModule(a_vulkan_device, &create_info, 
      nullptr, &(*shader_module));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create shader module (from '" << a_shader_path << 
            "')." << std::endl;
    return nullptr;
  }

  return shader_module;
}

std::shared_ptr<VkPipeline> GraphicsPipeline::GetVulkanPipeline() const
{
  return m_vulkan_graphics_pipeline;
}

std::shared_ptr<VkPipelineLayout> GraphicsPipeline::GetVulkanPipelineLayout() 
  const
{
  return m_vulkan_pipeline_layout;
}
