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

#ifndef INSTANCE_HPP
#define INSTANCE_HPP

#include <functional>
#include <memory>
#include <string>
#include <vector>

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

/**
 * @brief A wrapper for a Vulkan instance
 */
class Instance {
  public:
    Instance(std::string const &, bool);
    Instance(Instance const &) = delete;
    Instance &operator=(Instance const &) = delete;
    virtual ~Instance();
    std::vector<char const *> GetValidationLayers() const;
    std::shared_ptr<VkInstance> GetVulkanInstance() const;

  private:
    static VKAPI_ATTR VkBool32 VKAPI_CALL DebugCallback(VkDebugReportFlagsEXT,
        VkDebugReportObjectTypeEXT, uint64_t, uint64_t, int32_t, char const *, 
        char const *, void *);
    std::vector<char const *> GetRequiredExtensions();
    bool CheckValidationLayersSupport();
    int8_t CreateVulkanInstance(std::string const &);
    int8_t SetupVulkanDebugCallback();

    std::vector<char const *> m_validation_layers;
    std::shared_ptr<VkInstance> m_vulkan_instance;
    std::unique_ptr<VkDebugReportCallbackEXT,
      std::function<void(VkDebugReportCallbackEXT *)>> m_vulkan_debug_callback;
};

#endif
