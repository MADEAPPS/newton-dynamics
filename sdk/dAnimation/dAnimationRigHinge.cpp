/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "dAnimationStdAfx.h"
#include "dAnimationRigHinge.h"
#include "dAnimationCharacterRigManager.h"


dAnimationRigHinge::dAnimationRigHinge(dAnimationRigJoint* const parent, NewtonBody* const body)
	:dAnimationRigJoint(parent, body)
	,dCustomHinge (dGetIdentityMatrix(), body, parent->GetNewtonBody())
{
	dMatrix matrix;
	NewtonBodyGetMatrix(body, &matrix[0][0]);
	
	matrix = m_boneConvertionMatrix * matrix;
	CalculateLocalMatrix (matrix, m_localMatrix0, m_localMatrix1);
}

dAnimationRigHinge::~dAnimationRigHinge()
{
}


