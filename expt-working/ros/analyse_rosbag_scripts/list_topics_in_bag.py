#!/usr/bin/python2
import rosbag
import sys


filename = sys.argv[1]
print("Filename: " + filename)

bag = rosbag.Bag(filename)
topics = bag.get_type_and_topic_info()[1].keys()
print(str(topics))

types = []

for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
	print(i);
	types.append(bag.get_type_and_topic_info()[1].values()[i][0])

print(types)
