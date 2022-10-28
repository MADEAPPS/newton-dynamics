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

#include "ndNewton.h"
#include "ndMatrixGlue.h"
#include "ndVectorGlue.h"

ndMatrixGlue::ndMatrixGlue()
	:ndMatrix()
{
}

ndMatrixGlue::ndMatrixGlue(const ndMatrix& matrix)
	:ndMatrix(matrix)
{
}

ndMatrixGlue::ndMatrixGlue(const ndMatrixGlue& matrix)
	:ndMatrix(matrix)
{
}

ndMatrixGlue::ndMatrixGlue(const ndVectorGlue& front, const ndVectorGlue& up, const ndVectorGlue& right, const ndVectorGlue& posit)
	:ndMatrix(front, up, right, posit)
{
}

void ndMatrixGlue::SetIdentity()
{
	*this = ndGetIdentityMatrix();
}

ndVectorGlue ndMatrixGlue::Get(int i) const
{
	return ndVectorGlue((*this)[i]);
}

void ndMatrixGlue::Set(int i, ndVectorGlue& value)
{
	(*this)[i] = value;
}


