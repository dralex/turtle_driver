# -----------------------------------------------------------------------------
# The ROS2 Turtle Driver
# -----------------------------------------------------------------------------
#
# The ROS2 parameter declaration helper
#
# Copyright (C) 2026 Alexey Fedoseev <aleksey@fedoseev.net>
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see https://www.gnu.org/licenses/
#
# -----------------------------------------------------------------------------

from rcl_interfaces.msg import ParameterDescriptor


def declare(node, name, default, description):
    # declare a node parameter and return its value. The parameters are read once, while
    # the node is built; the topic names are not parameters, they are changed by the ROS2
    # remapping of the node
    node.declare_parameter(name, default, ParameterDescriptor(description=description))
    return node.get_parameter(name).value
