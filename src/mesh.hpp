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

#ifndef MESH_HPP
#define MESH_HPP

#include <memory>
#include <vector>
#include <string>

#include <vulkan/vulkan.h>

struct Vertex;

/**
 * @brief A mesh representation including vertices and indices.
 */
class Mesh {
  public:
    Mesh(std::string const &);
    Mesh(Mesh const &) = delete;
    Mesh &operator=(Mesh const &) = delete;
    virtual ~Mesh();
    std::shared_ptr<std::vector<uint32_t> const> GetIndices() const;
    std::shared_ptr<std::vector<Vertex> const> GetVertices() const;
  
  private:
    int8_t CreateMesh(std::string const &);

    std::shared_ptr<std::vector<Vertex>> m_vertices;
    std::shared_ptr<std::vector<uint32_t>> m_indices;
};

#endif
