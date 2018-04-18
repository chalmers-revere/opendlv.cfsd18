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

#include <instance.hpp>

Instance::Instance(std::string const &a_application_name, 
    bool a_enable_validation_layers):
  m_validation_layers(),
  m_vulkan_instance(),
  m_vulkan_debug_callback()
{
  if (a_enable_validation_layers) {
    m_validation_layers.push_back("VK_LAYER_LUNARG_standard_validation");
  }

  CreateVulkanInstance(a_application_name);
  SetupVulkanDebugCallback();
}

Instance::~Instance()
{
}

bool Instance::CheckValidationLayersSupport()
{
  uint32_t layer_count;
  vkEnumerateInstanceLayerProperties(&layer_count, nullptr);

  std::vector<VkLayerProperties> available_layers(layer_count);
  vkEnumerateInstanceLayerProperties(&layer_count, available_layers.data());
    
  for (char const *layer_name : m_validation_layers) {
    bool layer_found = false;

    for (auto const &layer_properties : available_layers) {
      if (strcmp(layer_name, layer_properties.layerName) == 0) {
        layer_found = true;
        break;
      }
    }
    if (!layer_found) {
      return false;
    }
  }

  return true;
}


int8_t Instance::CreateVulkanInstance(std::string const &a_application_name)
{
  bool enable_validation_layers = !m_validation_layers.empty();
  bool has_validation_layers_support = CheckValidationLayersSupport();
  if (enable_validation_layers && !has_validation_layers_support) {
    std::cerr << "Validation layers requested, but not available!" << std::endl;
    return -1;
  }

  VkApplicationInfo app_info = {};
  app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  app_info.pApplicationName = a_application_name.c_str();
  app_info.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
  app_info.pEngineName = "No Engine";
  app_info.engineVersion = VK_MAKE_VERSION(1, 0, 0);
  app_info.apiVersion = VK_API_VERSION_1_0;
        
  VkInstanceCreateInfo create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  create_info.pApplicationInfo = &app_info;
        
  auto extensions = GetRequiredExtensions();
  create_info.enabledExtensionCount = extensions.size();
  create_info.ppEnabledExtensionNames = extensions.data();

  if (enable_validation_layers) {
    create_info.enabledLayerCount = m_validation_layers.size();
    create_info.ppEnabledLayerNames = m_validation_layers.data();
  } else {
    create_info.enabledLayerCount = 0;
  }

  m_vulkan_instance = std::shared_ptr<VkInstance>(new VkInstance, 
      [](VkInstance *a_vulkan_instance) {
        vkDestroyInstance(*a_vulkan_instance, nullptr);
        std::cout << "DEBUG: Vulkan instance deleted." << std::endl;
      });

  int32_t res = vkCreateInstance(&create_info, nullptr, &(*m_vulkan_instance));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create instance." << std::endl;
    return -1;
  }

  return 0;
}

std::vector<char const *> Instance::GetRequiredExtensions()
{
  std::vector<char const *> extensions;

  uint32_t glfw_extension_count = 0;
  char const **glfw_extensions =
    glfwGetRequiredInstanceExtensions(&glfw_extension_count);

  for (uint32_t i = 0; i < glfw_extension_count; i++) {
    extensions.push_back(glfw_extensions[i]);
  }

  bool enable_validation_layers = !m_validation_layers.empty();
  if (enable_validation_layers) {
    extensions.push_back(VK_EXT_DEBUG_REPORT_EXTENSION_NAME);
  }

  return extensions;
}

std::vector<char const *> Instance::GetValidationLayers() const
{
  return m_validation_layers;
}

std::shared_ptr<VkInstance> Instance::GetVulkanInstance() const
{
  return m_vulkan_instance;
}

int8_t Instance::SetupVulkanDebugCallback()
{
  bool enable_validation_layers = !m_validation_layers.empty();
  if (!enable_validation_layers) {
    return 0;
  }

  VkDebugReportCallbackCreateInfoEXT create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_DEBUG_REPORT_CALLBACK_CREATE_INFO_EXT;
  create_info.flags = 
    VK_DEBUG_REPORT_ERROR_BIT_EXT | VK_DEBUG_REPORT_WARNING_BIT_EXT;
  create_info.pfnCallback = DebugCallback;

  m_vulkan_debug_callback = std::unique_ptr<VkDebugReportCallbackEXT, 
    std::function<void(VkDebugReportCallbackEXT *)>>(
      new VkDebugReportCallbackEXT,
      [&](VkDebugReportCallbackEXT *a_vulkan_debug_callback) {
        auto func = (PFN_vkDestroyDebugReportCallbackEXT) vkGetInstanceProcAddr(
            *m_vulkan_instance, "vkDestroyDebugReportCallbackEXT");
        if (func != nullptr) {
          func(*m_vulkan_instance, *a_vulkan_debug_callback, nullptr);
        }
        std::cout << "DEBUG: Debug callback deleted." << std::endl;
      });

  int32_t res;
  auto func = (PFN_vkCreateDebugReportCallbackEXT)
    vkGetInstanceProcAddr(*m_vulkan_instance, "vkCreateDebugReportCallbackEXT");
  if (func != nullptr) {
    res = func(*m_vulkan_instance, &create_info, nullptr, 
        &(*m_vulkan_debug_callback));
  } else {
    res = VK_ERROR_EXTENSION_NOT_PRESENT;
  }
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to set up debug callback." << std::endl;
    return -1;
  }

  return 0;
}

VKAPI_ATTR VkBool32 VKAPI_CALL Instance::DebugCallback(
    VkDebugReportFlagsEXT, VkDebugReportObjectTypeEXT, uint64_t, uint64_t, 
    int32_t, char const *, char const *a_msg, void *)
{
  std::cerr << "Validation layer message: " << a_msg << std::endl;
  return VK_FALSE;
}
