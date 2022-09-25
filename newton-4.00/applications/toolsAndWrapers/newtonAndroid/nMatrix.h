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

#ifndef _N_MATRIX_H_
#define _N_MATRIX_H_

#include "ndMatrix.h"
#include "nVector.h"

class nMatrix : public ndMatrix
{
	public:
	nMatrix()
		:ndMatrix()
	{
	}

	nMatrix(const ndMatrix& matrix)
		:ndMatrix(matrix)
	{
	}

	void SetIdentity()
	{
		*this = ndGetIdentityMatrix();
	}

	nVector Get(int i) const
	{
		return nVector((*this)[i]);
	}

	void Set(int i, nVector& value)
	{
		(*this)[i] = value;
	}
};

#endif 

