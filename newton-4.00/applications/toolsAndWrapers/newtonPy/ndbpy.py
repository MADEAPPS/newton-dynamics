# Copyright (c) <2003-2021> <Newton Game Dynamics>
 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

#import bpy
import newtonPy
dir(newtonPy)

class  Vector1(object):
	def __init__(self):
		self.x = 1.0
		self.y = 2.0
		self.z = 3.0

	def Show(self):
		print ('py  vector ', self.x, self.y, self.z) 

xxx = Vector1 ()
print ('py  vector ', xxx.x, xxx.y, xxx.z) 

xxx1 = newtonPy.dVector(1.0, 2.0, 3.0, 4.0)
print ('cpp vector ',  xxx1.GetX(), xxx1.GetY(), xxx1.GetZ())