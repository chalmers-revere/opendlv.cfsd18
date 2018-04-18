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

#include "commandpool.hpp"
#include "device.hpp"

CommandPool::CommandPool(Device const &a_device): 
  m_vulkan_command_pool()
{
  CreateVulkanCommandPool(a_device);
}

CommandPool::~CommandPool()
{
}

VkCommandBuffer CommandPool::BeginSingleTimeCommands(Device const &a_device) const
{
  auto vulkan_device = a_device.GetVulkanDevice();

  VkCommandBufferAllocateInfo allocate_info = {};
  allocate_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  allocate_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  allocate_info.commandPool = *m_vulkan_command_pool;
  allocate_info.commandBufferCount = 1;

  VkCommandBuffer command_buffer;
  vkAllocateCommandBuffers(*vulkan_device, &allocate_info, &command_buffer);

  VkCommandBufferBeginInfo begin_info = {};
  begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

  vkBeginCommandBuffer(command_buffer, &begin_info);

  return command_buffer;
}

int8_t CommandPool::CreateVulkanCommandPool(Device const &a_device)
{
  auto vulkan_device = a_device.GetVulkanDevice();
  auto vulkan_graphics_queue_family = a_device.GetVulkanGraphicsQueueFamily();

  VkCommandPoolCreateInfo create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  create_info.queueFamilyIndex = vulkan_graphics_queue_family;

  m_vulkan_command_pool = std::shared_ptr<VkCommandPool>(new VkCommandPool, 
      [vulkan_device](VkCommandPool *a_vulkan_command_pool) {
        vkDestroyCommandPool(*vulkan_device, *a_vulkan_command_pool, nullptr);
        std::cout << "DEBUG: Command pool deleted." << std::endl;
      });

  int32_t res = vkCreateCommandPool(*vulkan_device, &create_info, nullptr, 
      &(*m_vulkan_command_pool));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create command pool." << std::endl;
    return -1;
  }

  return 0;
}

void CommandPool::EndSingleTimeCommands(Device const &a_device, 
    VkCommandBuffer a_command_buffer) const {
  auto vulkan_device = a_device.GetVulkanDevice();
  auto vulkan_graphics_queue = a_device.GetVulkanGraphicsQueue();

  vkEndCommandBuffer(a_command_buffer);

  VkSubmitInfo submit_info = {};
  submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submit_info.commandBufferCount = 1;
  submit_info.pCommandBuffers = &a_command_buffer;

  vkQueueSubmit(*vulkan_graphics_queue, 1, &submit_info, VK_NULL_HANDLE);
  vkQueueWaitIdle(*vulkan_graphics_queue);

  vkFreeCommandBuffers(*vulkan_device, *m_vulkan_command_pool, 1, 
      &a_command_buffer);
}

std::shared_ptr<VkCommandPool> CommandPool::GetVulkanCommandPool() const
{
  return m_vulkan_command_pool;
}
