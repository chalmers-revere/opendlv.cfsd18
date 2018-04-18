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

#include <instance.hpp>
#include <surface.hpp>

Surface::Surface(GLFWwindow &a_window, Instance const &a_instance):
  m_vulkan_surface()
{
  auto vulkan_instance = a_instance.GetVulkanInstance();

  CreateVulkanSurface(a_window, *vulkan_instance);
}

Surface::~Surface()
{
}
    
std::shared_ptr<VkSurfaceKHR> Surface::GetVulkanSurface() const
{
  return m_vulkan_surface;
}

int8_t Surface::CreateVulkanSurface(GLFWwindow &a_window, 
    VkInstance const &a_vulkan_instance)
{
  m_vulkan_surface = std::shared_ptr<VkSurfaceKHR>(new VkSurfaceKHR,
      [a_vulkan_instance](VkSurfaceKHR *a_vulkan_surface) {
        vkDestroySurfaceKHR(a_vulkan_instance, *a_vulkan_surface, nullptr);
        std::cout << "DEBUG: Surface deleted." << std::endl;
      });

  int32_t res = glfwCreateWindowSurface(a_vulkan_instance, &a_window, 
      nullptr, &(*m_vulkan_surface));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create surface." << std::endl;
    return -1;
  }

  return 0;
}
