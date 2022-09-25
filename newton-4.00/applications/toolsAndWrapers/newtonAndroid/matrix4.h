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

#ifndef _ND_MATRIX4_H_
#define _ND_MATRIX4_H_

#include "ndMatrix.h"
#include "vector4.h"

namespace nd
{
	class Matrix4 : public ndMatrix
	{
		public:
		Matrix4()
			:ndMatrix()
		{
		}

		Matrix4(const ndMatrix& matrix)
			:ndMatrix(matrix)
		{
		}

		void SetIdentity()
		{
			*this = ndGetIdentityMatrix();
		}

		Vector4 Get(int i) const
		{
			return Vector4((*this)[i]);
		}

		void Set(int i, Vector4& value)
		{
			(*this)[i] = value;
		}
	};
}

#endif 

