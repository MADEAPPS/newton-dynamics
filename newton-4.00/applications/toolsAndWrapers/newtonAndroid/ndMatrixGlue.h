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

#ifndef _ND_MATRIX_GLUE_H_
#define _ND_MATRIX_GLUE_H_

#include "ndVectorGlue.h"
#include "ndMatrix.h"


class ndMatrixGlue : public ndMatrix
{
	public:
	ndMatrixGlue()
		:ndMatrix()
	{
	}

	ndMatrixGlue(const ndMatrix& matrix)
		:ndMatrix(matrix)
	{
	}

	ndMatrixGlue(const ndMatrixGlue& matrix)
		:ndMatrix(matrix)
	{
	}

	ndMatrixGlue(const ndVectorGlue& front, const ndVectorGlue& up, const ndVectorGlue& right, const ndVectorGlue& posit)
		:ndMatrix(front, up, right, posit)
	{
	}

	void SetIdentity()
	{
		*this = ndGetIdentityMatrix();
	}

	ndVectorGlue Get(int i) const
	{
		return ndVectorGlue((*this)[i]);
	}

	void Set(int i, ndVectorGlue& value)
	{
		(*this)[i] = value;
	}
};

#endif 

