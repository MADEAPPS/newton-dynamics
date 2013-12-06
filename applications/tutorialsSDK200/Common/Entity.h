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

#ifndef __ENTITY__H
#define __ENTITY__H

#include "dVector.h"
#include "dMatrix.h"
#include "dQuaternion.h"


class Entity
{
	public:
	struct SubMesh
	{
		unsigned m_textureHandle;
		int m_indexCount;
		unsigned short* m_indexArray;
	};

	Entity(void);
	virtual ~Entity(void);

	void OptimizeMesh();
	void LoadMesh (const char* name);
	virtual void Render (dFloat interpolationParam);
	void GetBBox (dVector& minBox, dVector& maxBox);

	// these are the element to represent the position and orientation state of a graphics object in the world 
	dMatrix m_matrix;					// current interpolated visual matrix
	dVector m_curPosition;				// position one physics simulation step in the future
	dVector m_prevPosition;             // position at the current physics simulation step
	dQuaternion m_curRotation;          // rotation one physics simulation step in the future  
	dQuaternion m_prevRotation;         // rotation at the current physics simulation step  


	// These are the elements for a simple vertex list index list graphics object. 
	int m_displayList;
	int m_subMeshCount;					
	int m_vertexCount;
	dFloat *m_uv;
	dFloat *m_vertex;
	dFloat *m_normal;
	SubMesh *m_subMeshes;
};


#endif
