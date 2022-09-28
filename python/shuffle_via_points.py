import random
import xml.etree.ElementTree as ET

# Read the original file
tree = ET.parse('dataset_1.posright_arm')
root = tree.getroot()

# Retrieve the number of via points
n_via_points = root.attrib['TotPositions']
print("Number of via points:", n_via_points)

# Retrieve the array of via points
via_points = []
for child in root:
    for sub_child in child:
        if sub_child.tag == "JointPositions":
            print(sub_child.tag)
            via_point = []
            for pos in sub_child:
                via_point.append(pos.text)
            print(via_point)
            via_points.append(via_point)
print("\nVia points:")
print(via_points)

# Shuffle the array of via points
random.shuffle(via_points)
print("\nShuffled via points:")
print(via_points)

# Update the via points in the file
for i in range(len(root)):
    child = root[i]
    via_point = via_points[i]
    for sub_child in child:
        if sub_child.tag == "JointPositions":
            for j in range(len(sub_child)):
                pos = sub_child[j]
                new_pos = via_point[j]
                pos.text = new_pos

# Check that the via points in the file have been updated
print("\nShuffled output:")
for child in root:
    for sub_child in child:
        if sub_child.tag == "JointPositions":
            print(sub_child.tag)
            via_point = []
            for pos in sub_child:
                via_point.append(pos.text)
            print(via_point)
            via_points.append(via_point)

# Save the updated file
tree.write('output.posright_arm')