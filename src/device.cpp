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
#include <limits>

#include "device.hpp"
#include "instance.hpp"
#include "surface.hpp"

Device::Device(Instance const &a_instance, Surface const &a_surface):
  m_validation_layers(a_instance.GetValidationLayers()),
  m_vulkan_device(),
  m_vulkan_graphics_queue(new VkQueue),
  m_vulkan_present_queue(new VkQueue),
  m_vulkan_physical_device(VK_NULL_HANDLE),
  m_vulkan_graphics_queue_family(),
  m_vulkan_present_queue_family()
{
  auto vulkan_instance = a_instance.GetVulkanInstance();
  auto vulkan_surface = a_surface.GetVulkanSurface();

  SelectVulkanPhysicalDevice(*vulkan_instance, *vulkan_surface);
  CreateVulkanLogicalDevice();
}

Device::~Device()
{
}

int8_t Device::CreateVulkanLogicalDevice()
{
  std::vector<char const *> const device_extensions = {
    VK_KHR_SWAPCHAIN_EXTENSION_NAME
  };

  std::vector<VkDeviceQueueCreateInfo> queue_create_infos;
  std::set<uint32_t> unique_queue_families = {m_vulkan_graphics_queue_family,
    m_vulkan_present_queue_family};

  float queue_priority = 1.0f;
  for (int32_t queue_family : unique_queue_families) {
    VkDeviceQueueCreateInfo queue_create_info = {};
    queue_create_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queue_create_info.queueFamilyIndex = queue_family;
    queue_create_info.queueCount = 1;
    queue_create_info.pQueuePriorities = &queue_priority;
    queue_create_infos.push_back(queue_create_info);
  }

  VkPhysicalDeviceFeatures device_features = {};
  device_features.samplerAnisotropy = VK_TRUE;

  VkDeviceCreateInfo create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;

  create_info.pQueueCreateInfos = queue_create_infos.data();
  create_info.queueCreateInfoCount = static_cast<uint32_t>(
      queue_create_infos.size());

  create_info.pEnabledFeatures = &device_features;

  create_info.enabledExtensionCount = device_extensions.size();
  create_info.ppEnabledExtensionNames = device_extensions.data();

  if (m_validation_layers.size() > 0) {
    create_info.enabledLayerCount = m_validation_layers.size();
    create_info.ppEnabledLayerNames = m_validation_layers.data();
  } else {
    create_info.enabledLayerCount = 0;
  }

  m_vulkan_device = std::shared_ptr<VkDevice>(new VkDevice, 
      [](VkDevice *a_vulkan_device) {
        vkDestroyDevice(*a_vulkan_device, nullptr);
        std::cout << "DEBUG: Vulkan logical device deleted." << std::endl;
      });

  int32_t res = vkCreateDevice(m_vulkan_physical_device, &create_info, nullptr, 
      &(*m_vulkan_device));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create logical device." << std::endl;
    return -1;
  }

  vkGetDeviceQueue(*m_vulkan_device, m_vulkan_graphics_queue_family, 0, 
      &(*m_vulkan_graphics_queue));
  vkGetDeviceQueue(*m_vulkan_device, m_vulkan_present_queue_family, 0, 
      &(*m_vulkan_present_queue));

  return 0;
}

int32_t Device::FindVulkanGraphicsQueueFamily(
    VkPhysicalDevice a_vulkan_physical_device)
{
  uint32_t queue_family_count = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(a_vulkan_physical_device, 
      &queue_family_count, nullptr);

  std::vector<VkQueueFamilyProperties> queue_families(
      queue_family_count);
  vkGetPhysicalDeviceQueueFamilyProperties(a_vulkan_physical_device, 
      &queue_family_count, queue_families.data());

  for (uint32_t i = 0; i < queue_families.size(); i++) {
    auto const &queue_family = queue_families[i];
    
    if (queue_family.queueCount > 0 
        && queue_family.queueFlags & VK_QUEUE_GRAPHICS_BIT) {
      return i;
    }
  }

  return -1;
}

int32_t Device::FindVulkanPresentQueueFamily(
    VkPhysicalDevice a_vulkan_physical_device,
    VkSurfaceKHR const &a_vulkan_surface)
{
  uint32_t queue_family_count = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(a_vulkan_physical_device,
      &queue_family_count, nullptr);

  std::vector<VkQueueFamilyProperties> queue_families(
      queue_family_count);
  vkGetPhysicalDeviceQueueFamilyProperties(a_vulkan_physical_device, 
      &queue_family_count, queue_families.data());

  for (uint32_t i = 0; i < queue_families.size(); i++) {
    auto const &queue_family = queue_families[i];
    
    VkBool32 surface_support = false;
    vkGetPhysicalDeviceSurfaceSupportKHR(a_vulkan_physical_device, i,
        a_vulkan_surface, &surface_support);
    if (queue_family.queueCount > 0 && surface_support) {
      return i;
    }
  }

  return -1;
}

uint32_t Device::FindMemoryTypeIndex(VkMemoryRequirements a_memory_requirements,
    VkMemoryPropertyFlags a_memory_properties) const
{
  VkPhysicalDeviceMemoryProperties memory_properties;
  vkGetPhysicalDeviceMemoryProperties(m_vulkan_physical_device, 
      &memory_properties);

  for (uint32_t i = 0; i < memory_properties.memoryTypeCount; i++) {
    if ((a_memory_requirements.memoryTypeBits & (1 << i)) && 
        (memory_properties.memoryTypes[i].propertyFlags & a_memory_properties) ==
        a_memory_properties) {
      return i;
    }
  }

  return std::numeric_limits<uint32_t>::max();
}

bool Device::FindSwapchainSupport(
    VkPhysicalDevice a_vulkan_physical_device)
{
  uint32_t extension_count;
  vkEnumerateDeviceExtensionProperties(a_vulkan_physical_device, nullptr,
      &extension_count, nullptr);

  std::vector<VkExtensionProperties> available_extensions(extension_count);
  vkEnumerateDeviceExtensionProperties(a_vulkan_physical_device, nullptr,
      &extension_count, available_extensions.data());

  for (auto const &extension : available_extensions) {
    if (strcmp(extension.extensionName, VK_KHR_SWAPCHAIN_EXTENSION_NAME) == 0) {
      return true;
    }
  }

  return false;
}

std::shared_ptr<VkDevice> Device::GetVulkanDevice() const
{
  return m_vulkan_device;
}

VkFormatProperties Device::GetVulkanFormatProperties(VkFormat a_format) const
{
  VkFormatProperties format_properties;
  vkGetPhysicalDeviceFormatProperties(m_vulkan_physical_device, a_format, 
      &format_properties);
  return format_properties;
}

std::shared_ptr<VkQueue> Device::GetVulkanGraphicsQueue() const
{
  return m_vulkan_graphics_queue;
}

uint32_t Device::GetVulkanGraphicsQueueFamily() const
{
  return m_vulkan_graphics_queue_family;
}

VkPhysicalDevice Device::GetVulkanPhysicalDevice() const
{
  return m_vulkan_physical_device;
}

std::shared_ptr<VkQueue> Device::GetVulkanPresentQueue() const
{
  return m_vulkan_present_queue;
}

uint32_t Device::GetVulkanPresentQueueFamily() const
{
  return m_vulkan_present_queue_family;
}

bool Device::HasMemoryType(VkMemoryRequirements a_memory_requirements,
    VkMemoryPropertyFlags a_memory_properties) const
{
  uint32_t memory_type_index = FindMemoryTypeIndex(a_memory_requirements,
      a_memory_properties);

  bool has_memory_type = 
    (memory_type_index != std::numeric_limits<uint32_t>::max());

  return has_memory_type;
}

int8_t Device::SelectVulkanPhysicalDevice(VkInstance const &a_vulkan_instance, 
    VkSurfaceKHR const &a_vulkan_surface)
{
  uint32_t physical_device_count = 0;
  vkEnumeratePhysicalDevices(a_vulkan_instance, &physical_device_count, 
      nullptr);

  if (physical_device_count == 0) {
    std::cerr << "Could not find any GPU with Vulkan support." << std::endl;
    return -1;
  }

  std::vector<VkPhysicalDevice> vulkan_physical_devices(physical_device_count);
  vkEnumeratePhysicalDevices(a_vulkan_instance, &physical_device_count, 
      vulkan_physical_devices.data());

  for (auto const &vulkan_physical_device : vulkan_physical_devices) {

    int32_t graphics_queue_family = FindVulkanGraphicsQueueFamily(
        vulkan_physical_device);
    int32_t present_queue_family = FindVulkanPresentQueueFamily(
        vulkan_physical_device, a_vulkan_surface);

    bool swapchain_support = FindSwapchainSupport(vulkan_physical_device);


    uint32_t format_count;
    vkGetPhysicalDeviceSurfaceFormatsKHR(vulkan_physical_device, 
        a_vulkan_surface, &format_count, nullptr);

    uint32_t present_mode_count;
    vkGetPhysicalDeviceSurfacePresentModesKHR(vulkan_physical_device, 
        a_vulkan_surface, &present_mode_count, nullptr);

    if (graphics_queue_family >= 0 &&
        present_queue_family >= 0 &&
        swapchain_support &&
        format_count > 0 &&
        present_mode_count > 0) {

      VkPhysicalDeviceProperties physical_properties = {};
      vkGetPhysicalDeviceProperties(vulkan_physical_device, 
          &physical_properties);
      std::cout << "GPU selected: " << physical_properties.deviceName 
        << std::endl;

      m_vulkan_physical_device = vulkan_physical_device;
      m_vulkan_graphics_queue_family = graphics_queue_family;
      m_vulkan_present_queue_family = present_queue_family; 
      break;
    }
  }

  if (m_vulkan_physical_device == VK_NULL_HANDLE) {
    std::cerr << "Could not find any GPU with support for this application." 
      << std::endl;
    return -1;
  }

  return 0;
}
