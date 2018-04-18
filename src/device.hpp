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

#ifndef DEVICE_HPP
#define DEVICE_HPP

#include <memory>
#include <iostream>
#include <set>
#include <vector>

#include <vulkan/vulkan.h>

class Instance;
class Surface;

/**
 * @brief A wrapper for a Vulkan device
 */
class Device {
  public:
    Device(Instance const &, Surface const &);
    Device(Device const &) = delete;
    Device &operator=(Device const &) = delete;
    virtual ~Device();
    uint32_t FindMemoryTypeIndex(VkMemoryRequirements, VkMemoryPropertyFlags) 
      const; 
    std::shared_ptr<VkDevice> GetVulkanDevice() const;
    VkFormatProperties GetVulkanFormatProperties(VkFormat) const;
    std::shared_ptr<VkQueue> GetVulkanGraphicsQueue() const;
    uint32_t GetVulkanGraphicsQueueFamily() const;
    VkPhysicalDevice GetVulkanPhysicalDevice() const;
    std::shared_ptr<VkQueue> GetVulkanPresentQueue() const;
    uint32_t GetVulkanPresentQueueFamily() const;
    bool HasMemoryType(VkMemoryRequirements, VkMemoryPropertyFlags) const; 

  
  private:
    int8_t CreateVulkanLogicalDevice();
    int32_t FindVulkanGraphicsQueueFamily(VkPhysicalDevice);
    int32_t FindVulkanPresentQueueFamily(VkPhysicalDevice, 
        VkSurfaceKHR const &);
    bool FindSwapchainSupport(VkPhysicalDevice);
    int8_t SelectVulkanPhysicalDevice(VkInstance const &, VkSurfaceKHR const &);

    std::vector<char const *> m_validation_layers;
    std::shared_ptr<VkDevice> m_vulkan_device;
    std::shared_ptr<VkQueue> m_vulkan_graphics_queue;
    std::shared_ptr<VkQueue> m_vulkan_present_queue;
    VkPhysicalDevice m_vulkan_physical_device;
    uint32_t m_vulkan_graphics_queue_family;
    uint32_t m_vulkan_present_queue_family;
};

#endif
