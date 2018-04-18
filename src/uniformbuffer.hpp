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

#ifndef UNIFORMBUFFER_HPP
#define UNIFORMBUFFER_HPP

#include <memory>

#include <vulkan/vulkan.h>

class Device;
class UniformBufferObject;

/**
 * @brief A wrapper for a Vulkan uniform buffer.
 */
class UniformBuffer {
  public:
    UniformBuffer(Device const &);
    UniformBuffer(UniformBuffer const &) = delete;
    UniformBuffer &operator=(UniformBuffer const &) = delete;
    virtual ~UniformBuffer();
    std::shared_ptr<VkBuffer> GetVulkanUniformBuffer() const;
    void UpdateUniformBufferObject(Device const &, UniformBufferObject const &);
  
  private:
    int8_t CreateVulkanUniformBuffer(Device const &);

    std::shared_ptr<VkBuffer> m_vulkan_uniform_buffer;
    std::shared_ptr<VkDeviceMemory> m_vulkan_uniform_buffer_memory;
};

#endif
