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

#ifndef COMMANDPOOL_HPP
#define COMMANDPOOL_HPP

#include <memory>

#include <vulkan/vulkan.h>

class Device;

/**
 * @brief A wrapper for a Vulkan command pool.
 */
class CommandPool {
  public:
    CommandPool(Device const &);
    CommandPool(CommandPool const &) = delete;
    CommandPool &operator=(CommandPool const &) = delete;
    virtual ~CommandPool();
    VkCommandBuffer BeginSingleTimeCommands(Device const &) const;
    void EndSingleTimeCommands(Device const &, VkCommandBuffer commandBuffer) 
      const;
    std::shared_ptr<VkCommandPool> GetVulkanCommandPool() const;
  
  private:
    int8_t CreateVulkanCommandPool(Device const &);

    std::shared_ptr<VkCommandPool> m_vulkan_command_pool;
};

#endif
