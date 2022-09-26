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

#ifndef _ND_BODY_NOTIFY_GLUE_H_
#define _ND_BODY_NOTIFY_GLUE_H_

#include "nMatrix.h"
#include "ndBodyNotify.h"

class ndBodyNotifyGlue : public ndBodyNotify
{
	public:
	ndBodyNotifyGlue()
		:ndBodyNotify(ndVector::m_zero)
	{
	}
	
	virtual void OnTransform(const nMatrix& matrix)
	{
	}
	
	virtual void OnTransform(ndInt32 threadIndex, const ndMatrix& matrix)
	{
		OnTransform(nMatrix(matrix));
	}

	void SetGravity(const nVector& gravity)
	{
		SetGravity(ndVector(gravity));
	}
};


#endif 

