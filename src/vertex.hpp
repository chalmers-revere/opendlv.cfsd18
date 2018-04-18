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

#ifndef VERTEX_HPP
#define VERTEX_HPP

#include <functional>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtx/hash.hpp>

struct Vertex {
  glm::vec3 position;
  glm::vec3 color;
  glm::vec2 texture;
    
  bool operator==(Vertex const &a_rhs) const
  {
    return position == a_rhs.position && color == a_rhs.color 
      && texture == a_rhs.texture;
  }
};

namespace std {
  template<> struct hash<Vertex> {
    size_t operator()(Vertex const &a_vertex) const {
      return ((hash<glm::vec3>()(a_vertex.position)
            ^ (hash<glm::vec3>()(a_vertex.color) << 1)) >> 1)
        ^ (hash<glm::vec2>()(a_vertex.texture) << 1);
    }
  };
}

#endif
