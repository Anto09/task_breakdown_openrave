#!/usr/bin/env python
from std_msgs.msg import Int64
from python_bindings_tutorial import AddTwoInts

if __name__ == "__main__":
	a = Int64(4)
	b = Int64(2)
	addtwoints = AddTwoInts()
	sum = addtwoints.add(a, b)
	print 'sum',sum