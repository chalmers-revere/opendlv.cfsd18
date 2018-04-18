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

#include "device.hpp"
#include "semaphore.hpp"

Semaphore::Semaphore(Device const &a_device): 
  m_vulkan_semaphore()
{
  auto vulkan_device = a_device.GetVulkanDevice();
  CreateVulkanSemaphore(*vulkan_device); 
}

Semaphore::~Semaphore()
{
}

int8_t Semaphore::CreateVulkanSemaphore(VkDevice const &a_vulkan_device)
{

  VkSemaphoreCreateInfo create_info = {};
  create_info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

  m_vulkan_semaphore = std::shared_ptr<VkSemaphore>(new VkSemaphore, 
      [a_vulkan_device](VkSemaphore *a_semaphore) {
        vkDestroySemaphore(a_vulkan_device, *a_semaphore, nullptr);
        std::cout << "DEBUG: Semaphore deleted." << std::endl;
      });

  uint32_t res = vkCreateSemaphore(a_vulkan_device, &create_info, nullptr, 
      &(*m_vulkan_semaphore));
  if (res != VK_SUCCESS) {
    std::cerr << "Failed to create semaphore." << std::endl;
    return -1;
  }

  return 0;
}

std::shared_ptr<VkSemaphore> Semaphore::GetVulkanSemaphore() const
{
  return m_vulkan_semaphore;
}
