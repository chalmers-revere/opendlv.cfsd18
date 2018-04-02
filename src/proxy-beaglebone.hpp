/*
 * Copyright (C) 2018  Christian Berger
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

#ifndef BEAGLEBONE
#define BEAGLEBONE

#include "opendlv-standard-message-set.hpp"

#include <string>
#include <utility>

class Beaglebone {
   private:
    Beaglebone(const Beaglebone &) = delete;
    Beaglebone(Beaglebone &&)      = delete;
    Beaglebone &operator=(const Beaglebone &) = delete;
    Beaglebone &operator=(Beaglebone &&) = delete;

   public:
    Beaglebone() = default;
    ~Beaglebone() = default;

   public:
    float decode(const std::string &data) noexcept;
};

#endif

