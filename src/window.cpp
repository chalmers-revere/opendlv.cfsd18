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

#include <iostream>

#include "window.hpp"

Window::Window():
    m_start_time(std::chrono::high_resolution_clock::now()),
    m_window(),
    m_vulkan(),
    m_title("OpenDLV Vulkan renderer"),
    m_height(600),
    m_width(800),
    m_initialized(false),
    m_running(false)
{
  if (!glfwInit()) {
    return;
  }

  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

  m_window = std::shared_ptr<GLFWwindow>(
      glfwCreateWindow(m_width, m_height, m_title.c_str(), nullptr, nullptr),
      [&](GLFWwindow *a_window) {
        glfwDestroyWindow(a_window);
        glfwTerminate();
        std::cout << "DEBUG: Window deleted." << std::endl;
      });

  glfwSetWindowUserPointer(&(*m_window), this);
  glfwSetKeyCallback(&(*m_window), OnKeyPress);
  glfwSetCursorPosCallback(&(*m_window), OnMouseCursorMove);
  glfwSetMouseButtonCallback(&(*m_window), OnMouseButtonPress);
  glfwSetWindowSizeCallback(&(*m_window), OnWindowResize);

  m_vulkan.reset(new Vulkan(m_window, m_title, m_width, m_height, 
        true));
}

Window::~Window()
{
}

Window& Window::GetInstance()
{
  Window static instance;
  return instance;
}

std::shared_ptr<Vulkan> Window::GetVulkan() const
{
  return m_vulkan;
}

void Window::OnKeyPress(GLFWwindow *, int32_t a_key, int32_t, int32_t a_action, 
    int32_t)
{
  Window::GetInstance().ParseGlfwButtonInput(a_key, a_action);
}

void Window::OnMouseButtonPress(GLFWwindow *, int32_t a_button, int32_t a_action,
    int32_t)
{
  Window::GetInstance().ParseGlfwButtonInput(a_button, a_action);
}

void Window::OnMouseCursorMove(GLFWwindow *, double a_x, double a_y)
{
  Window::GetInstance().ParseGlfwMouseCursorInput(a_x, a_y);
}

void Window::OnWindowResize(GLFWwindow *a_window, int32_t a_width, 
    int32_t a_height)
{
  if (a_width <= 0 || a_height <= 0) {
    return;
  }

  Window *app = reinterpret_cast<Window *>(glfwGetWindowUserPointer(a_window));
  app->GetVulkan()->RecreateSwapchain(a_width, a_height);
}

void Window::ParseGlfwButtonInput(int32_t a_button, int32_t)
{
  if (a_button == GLFW_KEY_ESCAPE) {
    m_running = false;
  }
}

void Window::ParseGlfwMouseCursorInput(double, double)
{
}

void Window::Start()
{
  m_running = true;

  while (!glfwWindowShouldClose(&(*m_window)) && m_running) {
    auto current_time = std::chrono::high_resolution_clock::now();
    double time = 
      std::chrono::duration_cast<std::chrono::milliseconds>(
          current_time - m_start_time).count() / 1000.0;
    m_vulkan->UpdateUniformBufferObject(time);
    m_vulkan->DrawFrame(m_width, m_height);

    glfwPollEvents();
  }
}
