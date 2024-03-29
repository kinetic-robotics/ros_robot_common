#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function, division
import xml.dom.minidom as xml
import argparse, os

def main(args):
    urdf = xml.parse(args.urdf_path)
    # 添加xmlns
    urdf.documentElement.setAttribute("xmlns:xacro", "http://www.ros.org/wiki/xacro")
    # 删除注释节点
    for node in urdf.childNodes:
        if node.nodeType == node.COMMENT_NODE:
            urdf.removeChild(node)
    for linkNode in urdf.getElementsByTagName("link"):
        linkName = linkNode.getAttribute("name")
        # 替换Collision
        collisionElement = urdf.createElement("collision")
        geometryElement  = urdf.createElement("geometry")
        xacroElement     = urdf.createElement("xacro:collision_" + linkName)
        geometryElement.appendChild(xacroElement)
        collisionElement.appendChild(geometryElement)
        try:
            linkNode.removeChild(linkNode.getElementsByTagName("collision")[0])
        except xml.dom.NotFoundErr:
            pass
        linkNode.appendChild(collisionElement)
    for jointNode in urdf.getElementsByTagName("joint"):
        # 修复safety_controller的BUG
        safetyElement = jointNode.getElementsByTagName("safety_controller")[0]
        if safetyElement.hasAttribute("soft_upper"):
            safetyElement.setAttribute("soft_upper_limit", safetyElement.getAttribute("soft_upper"))
            safetyElement.removeAttribute("soft_upper")
        if safetyElement.hasAttribute("soft_lower"):
            safetyElement.setAttribute("soft_lower_limit", safetyElement.getAttribute("soft_lower"))
            safetyElement.removeAttribute("soft_lower")
    if not os.path.exists(os.path.abspath(os.path.dirname(args.output_path) + "/.")):
        os.makedirs(os.path.abspath(os.path.dirname(args.output_path) + "/."))
    with open(args.output_path, "w") as file:
        urdf.writexml(file)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate xacro config from urdf generated by SolidWorks.')
    parser.add_argument('--urdf_path', help="The path of the URDF generated by SolidWorks.", required=True)
    parser.add_argument('--output_path', help="The output xacro file.", default="./robot-solidworks.xarco")
    args = parser.parse_args()
    main(args)