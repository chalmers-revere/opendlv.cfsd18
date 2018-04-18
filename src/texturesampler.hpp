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

#ifndef TEXTURESAMPLER_HPP
#define TEXTURESAMPLER_HPP

#include <memory>

#include <vulkan/vulkan.h>

class Device;

/**
 * @brief A wrapper for a Vulkan texture sampler.
 */
class TextureSampler {
  public:
    TextureSampler(Device const &);
    TextureSampler(TextureSampler const &) = delete;
    TextureSampler &operator=(TextureSampler const &) = delete;
    virtual ~TextureSampler();
    std::shared_ptr<VkSampler> GetVulkanTextureSampler() const;
  
  private:
    int8_t CreateVulkanTextureSampler(VkDevice const &);

    std::shared_ptr<VkSampler> m_vulkan_texture_sampler;
};

#endif
