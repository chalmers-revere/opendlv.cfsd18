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

#ifndef WINDOW_HPP
#define WINDOW_HPP

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include "vulkan.hpp"

/**
 * @brief Used for creating a GLFW window and to handle user input.
 */
class Window {
 public:
  Window(Window const &) = delete;
  Window &operator=(Window const &) = delete;
  virtual ~Window();
  std::shared_ptr<Vulkan> GetVulkan() const;
  void ParseGlfwButtonInput(int32_t, int32_t);
  void ParseGlfwMouseCursorInput(double, double);
  void Start();

  Window static &GetInstance();

 private:
  Window();

  void static OnKeyPress(GLFWwindow *, int32_t, int32_t, int32_t, int32_t);
  void static OnMouseButtonPress(GLFWwindow *, int32_t, int32_t, int32_t);
  void static OnMouseCursorMove(GLFWwindow *, double, double);
  void static OnWindowResize(GLFWwindow *, int32_t, int32_t);

  std::chrono::time_point<std::chrono::high_resolution_clock> m_start_time;
  std::shared_ptr<GLFWwindow> m_window;
  std::shared_ptr<Vulkan> m_vulkan;
  std::string m_title;
  uint32_t m_height;
  uint32_t m_width;
  bool m_initialized;
  bool m_running;
};

#endif
