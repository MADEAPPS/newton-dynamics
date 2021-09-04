# Copyright (c) <2003-2021> <Newton Game Dynamics>
 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

#import bpy
import newtonPy


class  Vector1(object):
	def __init__(self):
		self.x = 1
		self.y = 2
		self.z = 3
		print ('esto es una mierda') 

	def Show(self):
		print (self.x, self.y, self.z) 

xxx = Vector1 ()
xxx.Show()

#xxx = newtonPy.dVector()
#xxx.SetX(1)
#xxx.GetX()