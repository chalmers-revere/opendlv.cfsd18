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
#include <unordered_map>
#include <utility>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include "mesh.hpp"
#include "vertex.hpp"

Mesh::Mesh(std::string const &a_mesh_path): 
  m_vertices(new std::vector<Vertex>()),
  m_indices(new std::vector<uint32_t>())
{
  CreateMesh(a_mesh_path); 
}

Mesh::~Mesh()
{
}

int8_t Mesh::CreateMesh(std::string const &a_mesh_path)
{
  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string error;

  int32_t res = tinyobj::LoadObj(&attrib, &shapes, &materials, &error, 
      a_mesh_path.c_str());
  if (!res) {
    std::cerr << "Failed to load mesh: " << error << std::endl;
    return -1;
  }

  std::unordered_map<Vertex, uint32_t> unique_vertices = {};

  for (auto const &shape : shapes) {
    for (auto const &index : shape.mesh.indices) {
      Vertex vertex = {};

      vertex.position = {
        attrib.vertices[3 * index.vertex_index + 0],
        attrib.vertices[3 * index.vertex_index + 1],
        attrib.vertices[3 * index.vertex_index + 2]
      };

      vertex.texture = {
        attrib.texcoords[2 * index.texcoord_index + 0],
        1.0f - attrib.texcoords[2 * index.texcoord_index + 1]
      };

      vertex.color = {1.0f, 1.0f, 1.0f};

      if (unique_vertices.count(vertex) == 0) {
        unique_vertices[vertex] = static_cast<uint32_t>(m_vertices->size());
        m_vertices->push_back(vertex);
      }

      m_indices->push_back(unique_vertices[vertex]);
    }
  }

  return 0;
}

std::shared_ptr<std::vector<uint32_t> const> Mesh::GetIndices() const
{
  return m_indices;
}

std::shared_ptr<std::vector<Vertex> const> Mesh::GetVertices() const
{
  return m_vertices;
}
