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

#ifndef __D_INVERSE_KINEMATIC_H__
#define __D_INVERSE_KINEMATIC_H__


#include "dContainersStdAfx.h"
#include "dContainersAlloc.h"

class dInverseKinematic: public dContainersAlloc
{
	public:
	class dIKNode;

	DCONTAINERS_API dInverseKinematic();
	DCONTAINERS_API ~dInverseKinematic();

	DCONTAINERS_API dIKNode* GetRoot() const;
	DCONTAINERS_API dIKNode* AttachLink(dIKNode* const parent) const;

	DCONTAINERS_API void SetAxis(dIKNode* const node, const dVector& pin) const;
	DCONTAINERS_API void SetPivot(dIKNode* const node, const dVector& pivot) const;

	protected:
	dIKNode* m_root;
};
#endif

