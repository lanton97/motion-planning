import xml.etree.ElementTree as ET
from xml.dom import minidom

# Simple script to generate a test maze

root = ET.Element('root')

start_pos = ET.SubElement(root, 'start_position')
start_pos.set('pos', '-90 90')

goal_pos = ET.SubElement(root, 'goal_position')
goal_pos.text = '90 -90'
goal_pos.set('pos', '90 -90')

walls = ET.SubElement(root, 'walls')

wall1 = ET.SubElement(walls, 'wall1')
wall1.set('start', '-40 0')
wall1.set('end', '-40 50')

wall2 = ET.SubElement(walls, 'wall2')
wall2.set('start', '0 40')
wall2.set('end', '-80 40')

tree = ET.ElementTree(root)
dom = minidom.parseString(ET.tostring(root))

xml_str = dom.toprettyxml(indent ="\t")

f = open("test_maze2.xml", "w+")

f.write(xml_str)
