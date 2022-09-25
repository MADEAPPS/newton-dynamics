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

#ifndef _ND_NATIVE_TYPES_H_
#define _ND_NATIVE_TYPES_H_

#include "ndTypes.h"

class int32
{
	public:
	int32(int value = 0)
		:m_value(value)
	{
	}

	int& operator()()
	{
		return m_value;
	}

	const int operator()() const
	{
		return m_value;
	}

	ndInt32 m_value;
};


class float32
{
	public:
	float32(float value = 0)
		:m_value(value)
	{
	}

	float& operator()()
	{
		return m_value;
	}

	const float operator()() const
	{
		return m_value;
	}

	ndFloat32 m_value;
};

#endif 

