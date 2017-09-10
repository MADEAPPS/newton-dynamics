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



// LevelPrimitive.h: interface for the LevelPrimitive class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_LEVELPRIMITIVE_H__211387E2_8B23_49AB_9F18_1746C49EBFD0__INCLUDED_)
#define AFX_LEVELPRIMITIVE_H__211387E2_8B23_49AB_9F18_1746C49EBFD0__INCLUDED_

#include "Newton.h"
#include "../OGLModel.h"


class LevelPrimitive: public OGLModel  
{
	public:
	LevelPrimitive(const char* name, NewtonWorld* nWorld, int optimized);
	virtual ~LevelPrimitive();
	NewtonBody* GetRigidBody() const {return m_level;}

	private:
//	static void Destructor (const NewtonBody* body);
	static void DebugCallback (const NewtonBody* bodyWithTreeCollision, const NewtonBody* body,	int faceID, int vertexCount, const dFloat* vertex, int vertexstrideInBytes);

	public:
	NewtonBody* m_level;
};

#endif 

