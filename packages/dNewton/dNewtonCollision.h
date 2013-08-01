/* Copyright (c) <2003-2013> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef _D_NEWTON_COLLISION_H_
#define _D_NEWTON_COLLISION_H_

#include "dStdAfxNewton.h"
#include "dNewtonAlloc.h"

class dNewton;
class dNewtonMesh;

class dNewtonCollision: public dNewtonAlloc
{
	public:
	enum dCollsionType
	{
		m_null,
		m_box,
		m_sphere,
		m_capsule,
		m_taperedCapsule,
		m_cone,
		m_cylinder,
		m_taperedCyinder,
		m_chamferedCylinder,
		m_convex,
		m_compound,
		
		m_mesh,
		m_scene,
		m_heighfield,
	};

	class dDebugRenderer
	{
		public:
		dDebugRenderer (dNewtonCollision* const me)
			:m_collision(me)
		{
		}
		
		virtual void OnDrawFace (int vertexCount, const dFloat* const faceVertex, int id) = NULL;

		dNewtonCollision* m_collision;
	};

	CNEWTON_API dNewtonCollision (dCollsionType type);
	CNEWTON_API virtual ~dNewtonCollision();

	CNEWTON_API NewtonCollision* GetShape() const;
	virtual dNewtonCollision* Clone (NewtonCollision* const shape) const = 0; 

	CNEWTON_API void* GetUserData() const;
	CNEWTON_API void SetUserData(void* const userData);

	CNEWTON_API dFloat GetVolume () const;

	CNEWTON_API void SetScale(dFloat x, dFloat y, dFloat z);
	CNEWTON_API void GetScale(dFloat& x, dFloat& y, dFloat& z) const;

	CNEWTON_API void SetMatrix (const dFloat* const matrix);
	CNEWTON_API void GetMatrix (dFloat* const matrix) const;

	CNEWTON_API void CalculateAABB (const dFloat* const matrix, dFloat* const p0, dFloat* const p1) const;
	CNEWTON_API virtual void DebugRender (const dFloat* const matrix, dDebugRenderer* const renderer) const;

	CNEWTON_API void CalculateBuoyancyAcceleration (const dFloat* const matrix, const dFloat* const shapeOrigin, const dFloat* const gravityVector, const dFloat* const fluidPlane, dFloat fluidDensity, dFloat fluidViscosity, dFloat* const accel, dFloat* const alpha);

	protected:
	CNEWTON_API dNewtonCollision (const dNewtonCollision& srcCollision, NewtonCollision* const shape);
	CNEWTON_API void SetShape (NewtonCollision* const shape) ;
	CNEWTON_API static void DebugRender (void* userData, int vertexCount, const dFloat* faceVertec, int id);
	

	NewtonCollision* m_shape;
	void* m_userData;
	dCollsionType m_type;
	friend dNewton;
};




class dNewtonCollisionMesh: public dNewtonCollision
{
	public: 
	CNEWTON_API dNewtonCollisionMesh (dNewton* const world);
	CNEWTON_API dNewtonCollisionMesh (dNewton* const world, const dNewtonMesh& mesh, int id);

	dNewtonCollision* Clone (NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionMesh (*this, shape);
	}

	CNEWTON_API virtual void BeginFace();
	CNEWTON_API virtual void AddFace(int vertexCount, const dFloat* const vertexPtr, int strideInBytes, int faceAttribute);
	CNEWTON_API virtual void EndFace();

	protected:
	dNewtonCollisionMesh (const dNewtonCollisionMesh& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionScene: public dNewtonCollision
{
	public: 
	CNEWTON_API dNewtonCollisionScene (dNewton* const world);
	dNewtonCollision* Clone (NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionScene (*this, shape);
	}

	CNEWTON_API virtual void BeginAddRemoveCollision();
	CNEWTON_API virtual void* AddCollision(const dNewtonCollision* const collision);
	CNEWTON_API virtual void RemoveCollision (void* const handle);
	CNEWTON_API virtual void EndAddRemoveCollision();

	protected:
	dNewtonCollisionScene (const dNewtonCollisionScene& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionHeightField: public dNewtonCollision
{
	public: 
	dNewtonCollisionHeightField (dNewton* const world, int width, int height, int gridsDiagonals, int elevationdataType, dFloat vertcalScale, dFloat horizontalScale, const void* const elevationMap, const char* const attributeMap, int shapeID)
		:dNewtonCollision(m_heighfield)
	{
		SetShape (NewtonCreateHeightFieldCollision (world->GetNewton(), width, height, gridsDiagonals, elevationdataType, elevationMap, attributeMap, vertcalScale, horizontalScale, shapeID));
	}

	dNewtonCollision* Clone (NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionHeightField (*this, shape);
	}

	protected:
	dNewtonCollisionHeightField (const dNewtonCollisionHeightField& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionNull: public dNewtonCollision
{
	public: 
	dNewtonCollisionNull (dNewton* const world)
		:dNewtonCollision(m_null)
	{
		SetShape (NewtonCreateNull(world->GetNewton()));
	}

	dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionNull (*this, shape);
	}

	protected:
	dNewtonCollisionNull (const dNewtonCollisionNull& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionBox: public dNewtonCollision
{
	public: 
	dNewtonCollisionBox (dNewton* const world, dFloat x, dFloat y, dFloat z, int id)
		:dNewtonCollision(m_box)
	{
		SetShape (NewtonCreateBox(world->GetNewton(), x, y, z, id, NULL));
	}

	dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionBox (*this, shape);
	}

	protected:
	dNewtonCollisionBox (const dNewtonCollisionBox& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionSphere: public dNewtonCollision
{
	public: 
	dNewtonCollisionSphere (dNewton* const world, dFloat radio, int id)
		:dNewtonCollision(m_sphere)
	{
		SetShape (NewtonCreateSphere(world->GetNewton(), radio, id, NULL));
	}

	dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionSphere (*this, shape);
	}

	protected:
	dNewtonCollisionSphere (const dNewtonCollisionSphere& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionCapsule: public dNewtonCollision
{
	public: 
	dNewtonCollisionCapsule (NewtonCollision* const shape)
		:dNewtonCollision(m_capsule)
	{
		SetShape (shape);
	}

	dNewtonCollisionCapsule (dNewton* const world, dFloat radio, dFloat height, int id)
		:dNewtonCollision(m_capsule)
	{
		SetShape (NewtonCreateCapsule (world->GetNewton(), radio, height, id, NULL));
	}

	dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionCapsule (*this, shape);
	}

	protected:
	dNewtonCollisionCapsule (const dNewtonCollisionCapsule& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};



class dNewtonCollisionTaperedCapsule: public dNewtonCollision
{
	public: 
	dNewtonCollisionTaperedCapsule (dNewton* const world, dFloat radio0, dFloat radio1, dFloat height, int id)
		:dNewtonCollision(m_taperedCapsule)
	{
		SetShape (NewtonCreateTaperedCapsule (world->GetNewton(), radio0, radio1, height, id, NULL));
	}

	dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionTaperedCapsule (*this, shape);
	}

	protected:
	dNewtonCollisionTaperedCapsule (const dNewtonCollisionTaperedCapsule& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionCone: public dNewtonCollision
{
	public: 
	dNewtonCollisionCone (dNewton* const world, dFloat radio, dFloat height, int id)
		:dNewtonCollision(m_cone)
	{
		SetShape (NewtonCreateCone (world->GetNewton(), radio, height, id, NULL));
	}

	dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionCone (*this, shape);
	}

	protected:
	dNewtonCollisionCone (const dNewtonCollisionCone& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionCylinder: public dNewtonCollision
{
	public: 
	dNewtonCollisionCylinder (NewtonCollision* const shape)
		:dNewtonCollision(m_cylinder)
	{
		SetShape (shape);
	}

	dNewtonCollisionCylinder (dNewton* const world, dFloat radio, dFloat height, int id)
		:dNewtonCollision(m_cylinder)
	{
		SetShape (NewtonCreateCylinder (world->GetNewton(), radio, height, id, NULL));
	}

	dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionCylinder (*this, shape);
	}

	protected:
	dNewtonCollisionCylinder (const dNewtonCollisionCylinder& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionTaperedCylinder: public dNewtonCollision
{
	public: 
	dNewtonCollisionTaperedCylinder (dNewton* const world, dFloat radio0, dFloat radio1, dFloat height, int id)
		:dNewtonCollision(m_taperedCyinder)
	{
		SetShape (NewtonCreateTaperedCylinder(world->GetNewton(), radio0, radio1, height, id, NULL));
	}

	dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionTaperedCylinder (*this, shape);
	}

	protected:
	dNewtonCollisionTaperedCylinder (const dNewtonCollisionTaperedCylinder& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionChamferedCylinder: public dNewtonCollision
{
	public: 
	dNewtonCollisionChamferedCylinder (dNewton* const world, dFloat radio, dFloat height, int id)
		:dNewtonCollision(m_chamferedCylinder)
	{
		SetShape (NewtonCreateChamferCylinder (world->GetNewton(), radio, height, id, NULL));
	}

	dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionChamferedCylinder (*this, shape);
	}

	protected:
	dNewtonCollisionChamferedCylinder (const dNewtonCollisionChamferedCylinder& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionConvexHull: public dNewtonCollision
{
	public: 
	dNewtonCollisionConvexHull (NewtonCollision* const shape)
		:dNewtonCollision(m_convex)
	{
		SetShape (shape);
	}

	CNEWTON_API dNewtonCollisionConvexHull (dNewton* const world, const dNewtonMesh& mesh, int id);


	dNewtonCollisionConvexHull (dNewton* const world, int vertexCount, const dFloat* const vertexCloud, int strideInBytes, dFloat tolerance, int shapeID)
		:dNewtonCollision(m_convex)
	{
		SetShape (NewtonCreateConvexHull (world->GetNewton(), vertexCount, vertexCloud, strideInBytes, tolerance, shapeID, NULL));
	}

	dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionConvexHull (*this, shape);
	}

	protected:
	dNewtonCollisionConvexHull (const dNewtonCollisionConvexHull& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionCompound: public dNewtonCollision
{
	public: 
	dNewtonCollisionCompound (NewtonCollision* const shape)
		:dNewtonCollision(m_compound)
	{
		SetShape (shape);
	}

	dNewtonCollisionCompound (dNewton* const world, int shapeID)
		:dNewtonCollision(m_compound)
	{
		SetShape (NewtonCreateCompoundCollision (world->GetNewton(), shapeID));
	}

	CNEWTON_API dNewtonCollisionCompound (dNewton* const world, const dNewtonMesh& mesh, int shapeID);


	dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionCompound (*this, shape);
	}

	CNEWTON_API virtual void BeginAddRemoveCollision();
	CNEWTON_API virtual void* AddCollision(const dNewtonCollision* const collision);
	CNEWTON_API virtual void RemoveCollision (void* const handle);
	CNEWTON_API virtual void EndAddRemoveCollision();

	protected:
	dNewtonCollisionCompound (const dNewtonCollisionCompound& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};



#endif
