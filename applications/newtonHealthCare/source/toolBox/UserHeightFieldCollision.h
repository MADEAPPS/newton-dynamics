/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// UserHeightFieldCollision.h: interface for the UserHeightFieldCollision class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_UserHeightFieldCollision_H__6518E22E_3B44_441D_BB92_9732FF065C7B__INCLUDED_)
#define AFX_UserHeightFieldCollision_H__6518E22E_3B44_441D_BB92_9732FF065C7B__INCLUDED_



#include "Newton.h"
#include "RenderPrimitive.h"

#define HEIGHT_SIZE			256

#define CELL_SIZE			8.0f
#define HIGHTSCALE_SIZE		1.0f
#define TEXTURE_SCALE		(1.0f / 16.0f)

#define MAX_COLLIDING_FACES	128

#define MAX_THREAD_FACES	8

class UserHeightFieldCollision : public RenderPrimitive   
{
	public:
	UserHeightFieldCollision(NewtonWorld* nWorld);
	virtual ~UserHeightFieldCollision();

	NewtonBody* GetRigidBody() const;

	dFloat* GetElevationMap () { return &m_heightField[0][0];}

	private:

	dFloat RayCastTriangle (const dVector& p0, const dVector& dp, const dVector& origin, const dVector& e1, const dVector& e2);
	dFloat RayCastCell (dInt32 xIndex0, dInt32 zIndex0, const dVector& p0, const dVector& dp, dVector& normalOut);
	void CalculateMinExtend2d (const dVector& p0, const dVector& p1, dVector& boxP0, dVector& boxP1);
	void CalculateMinExtend3d (const dVector& p0, const dVector& p1, dVector& boxP0, dVector& boxP1);
	bool ClipRay2d (dVector& p0, dVector& p1, const dVector& boxP0, const dVector& boxP1); 

	

	NewtonBody* m_level;
	dVector m_minBox;
	dVector m_maxBox;
	dFloat m_heightField[HEIGHT_SIZE][HEIGHT_SIZE];
	
	dInt32 m_attribute[MAX_THREAD_FACES][MAX_COLLIDING_FACES];
	dInt32 m_faceIndices[MAX_THREAD_FACES][MAX_COLLIDING_FACES];
	dInt32 m_indexArray[MAX_THREAD_FACES][MAX_COLLIDING_FACES * 3];
	dVector m_collisionVertex[MAX_THREAD_FACES][MAX_COLLIDING_FACES * 2];

	static void MeshCollisionCollideCallback (NewtonUserMeshCollisionCollideDesc* collideDescData);
	static dFloat UserMeshCollisionRayHitCallback (NewtonUserMeshCollisionRayHitDesc* lineDescData);
};

#endif 

