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

#ifndef _SHAPE_H_
#define _SHAPE_H_

#include "ndShape.h"

namespace nd
{
	class Shape
	{
		public:
		ndShape* GetShape() const
		{
			return m_shape;
		}
		
		protected:
		Shape(ndShape* const shape)
			:m_shape(shape)
		{
		}

		virtual ~Shape()
		{
		}

		ndShape* m_shape;
	};
}

#endif 

