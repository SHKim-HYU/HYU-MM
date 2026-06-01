#!/usr/bin/env python3
# Emit the NOMINAL (green "ghost") robot_description from a URDF:
#   1) prefix every LINK name (and joint parent/child link refs) with "nominal/"
#      so robot_state_publisher emits nominal/base_link, nominal/link0, ...
#      WITHOUT relying on frame_prefix (ROS1 Noetic robot_state_publisher ignores
#      it / tf_prefix was removed). JOINT names are left unprefixed so the node's
#      /nominal/joint_states (joint0..joint5) still drives the arm.
#   2) tint every visual light-green (RViz Alpha gives the transparency).
#
# The URDF path is the first positional arg:
#   prepare_urdf.py <urdf_path>   -> green + nominal/-prefixed robot_description
import re
import sys

args = [a for a in sys.argv[1:] if not a.startswith("-")]
if not args:
    sys.stderr.write("usage: prepare_urdf.py <urdf_path>\n")
    sys.exit(1)

xml = open(args[0]).read()

# --- 1) prefix link names + joint parent/child link references (NOT joint names) ---
xml = re.sub(r'(<link\s+name=")',   r'\1nominal/', xml)
xml = re.sub(r'(<parent\s+link=")', r'\1nominal/', xml)
xml = re.sub(r'(<child\s+link=")',  r'\1nominal/', xml)

# --- 2) green tint: drop existing materials, add one to every visual ---
xml = re.sub(r'<robot\s+name="[^"]*"', '<robot name="hyumm_scan_nominal"', xml, count=1)
xml = re.sub(r'<material\b[^>]*?/>', '', xml)
xml = re.sub(r'<material\b.*?</material>', '', xml, flags=re.S)
xml = xml.replace('</visual>',
                  '<material name="nominal_green">'
                  '<color rgba="0.45 1.0 0.45 0.35"/></material></visual>')

sys.stdout.write(xml)
