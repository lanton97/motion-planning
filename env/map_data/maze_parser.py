import xml.etree.ElementTree as ET

# Transform the position string into a list of the positions
def posStrIntoList(pos_str):
    x_pos, y_pos = pos_str.split()
    return [float(x_pos), float(y_pos)]

# Check if the xml tree has the valid nodes
def isValidMazeXML(file_path):
    tree = ET.parse(file_path)
    if tree is None:
        return False
    if tree.find('start_position') is None or \
       tree.find('goal_position') is None or \
       tree.find('walls') is None:
        return False
    return True

# Extract key elements of the maze from the XML file and return them
# Elements include the start and goal positions, as well as wall
# information
def parseMazeXML(file_path):
    tree = ET.parse(file_path)

    start_node = tree.find('start_position')
    start_pos = posStrIntoList(start_node.attrib['pos'])

    end_node = tree.find('goal_position')
    end_pos = posStrIntoList(start_node.attrib['pos'])

    walls_node  = tree.find('walls')
    walls = []
    # Iterate through the child nodes of the walls object
    for wall in walls_node.iter():
        # Skip the walls node itself
        if wall is not walls_node:
            wall_start = posStrIntoList(wall.attrib['start'])
            wall_end = posStrIntoList(wall.attrib['end'])
            walls.append([wall_start, wall_end])

    return start_pos, end_pos, walls

