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

#ifndef SURFACE_HPP
#define SURFACE_HPP

#include <memory>
#include <iostream>

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

class Instance;

/**
 * @brief A wrapper for a Vulkan surface
 */
class Surface {
  public:
    Surface(GLFWwindow &, Instance const &);
    Surface(Surface const &) = delete;
    Surface &operator=(Surface const &) = delete;
    virtual ~Surface();
    std::shared_ptr<VkSurfaceKHR> GetVulkanSurface() const;
  
  private:
    int8_t CreateVulkanSurface(GLFWwindow &, VkInstance const &);

    std::shared_ptr<VkSurfaceKHR> m_vulkan_surface;
};

#endif
