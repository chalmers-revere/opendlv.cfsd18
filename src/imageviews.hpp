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

#ifndef IMAGEVIEWS_HPP
#define IMAGEVIEWS_HPP

#include <vulkan/vulkan.h>

class Device;
class Swapchain;

/**
 * @brief A wrapper for a Vulkan image views
 */
class ImageViews {
  public:
    ImageViews(Device const &, Swapchain const &);
    ImageViews(ImageViews const &) = delete;
    ImageViews &operator=(ImageViews const &) = delete;
    virtual ~ImageViews();
    std::shared_ptr<VkImageView> GetVulkanImageView(uint32_t) const;
    uint32_t GetImageViewCount() const;
  
  private:
    int8_t CreateVulkanImageViews(VkDevice const &,
        std::shared_ptr<std::vector<VkImage>>, VkFormat);

    std::vector<std::shared_ptr<VkImageView>> m_vulkan_image_views;
};

#endif
