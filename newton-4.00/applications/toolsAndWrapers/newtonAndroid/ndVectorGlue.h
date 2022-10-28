/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef _ND_VECTOR_GLUE_H_
#define _ND_VECTOR_GLUE_H_

#include "ndVector.h"

class ndVectorGlue : public ndVector
{
	public:
	ndVectorGlue();
	ndVectorGlue(const ndVector& vector);
	ndVectorGlue(const ndVectorGlue& vector);
	ndVectorGlue(float x, float y, float z, float w);

	float Get(int i) const;
	void Set(int i, float value);
};



#endif 

