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
#include "dNewtonMaterial.h"

class dNewton;
class dNewtonMesh;

class dNewtonCollision: virtual public dNewtonAlloc, public dNewtonMaterial
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
		CNEWTON_API dDebugRenderer (dNewtonCollision* const me)
			:m_collision(me)
		{
		}
		
		CNEWTON_API virtual void OnDrawFace (int vertexCount, const dFloat* const faceVertex, int faceId) = NULL;

		dNewtonCollision* m_collision;
	};

	CNEWTON_API dNewtonCollision (dCollsionType type, dLong collisionMask);
	CNEWTON_API virtual ~dNewtonCollision();

	CNEWTON_API dCollsionType GetType() const {return m_type;}
	CNEWTON_API NewtonCollision* GetShape() const;
	CNEWTON_API virtual dNewtonCollision* Clone (NewtonCollision* const shape) const = 0; 

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
	friend class dNewton;
};




class dNewtonCollisionMesh: public dNewtonCollision
{
	public: 
	CNEWTON_API dNewtonCollisionMesh (dNewton* const world, dLong collisionMask);
	CNEWTON_API dNewtonCollisionMesh (dNewton* const world, const dNewtonMesh& mesh, dLong collisionMask);

	CNEWTON_API dNewtonCollision* Clone (NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionMesh (*this, shape);
	}

	CNEWTON_API virtual void BeginFace();
	CNEWTON_API virtual void AddFace(int vertexCount, const dFloat* const vertexPtr, int strideInBytes, int faceAttribute);
	CNEWTON_API virtual void EndFace();

	protected:
	CNEWTON_API dNewtonCollisionMesh (const dNewtonCollisionMesh& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionScene: public dNewtonCollision
{
	public: 
	CNEWTON_API dNewtonCollisionScene (dNewton* const world, dLong collisionMask);
	CNEWTON_API dNewtonCollision* Clone (NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionScene (*this, shape);
	}

	CNEWTON_API virtual void BeginAddRemoveCollision();
	CNEWTON_API virtual void* AddCollision(const dNewtonCollision* const collision);
	CNEWTON_API virtual void RemoveCollision (void* const handle);
	CNEWTON_API virtual void EndAddRemoveCollision();

	CNEWTON_API void* GetFirstNode () const;;
	CNEWTON_API void* GetNextNode (void* const collisionNode) const;
	CNEWTON_API dNewtonCollision* GetChildFromNode(void* const collisionNode) const; 

	protected:
	CNEWTON_API dNewtonCollisionScene (const dNewtonCollisionScene& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionHeightField: public dNewtonCollision
{
	public: 
	CNEWTON_API dNewtonCollisionHeightField (dNewton* const world, int width, int height, int gridsDiagonals, int elevationdataType, dFloat vertcalScale, dFloat horizontalScale, const void* const elevationMap, const char* const attributeMap, dLong collisionMask)
		:dNewtonCollision(m_heighfield, collisionMask)
	{
		SetShape (NewtonCreateHeightFieldCollision (world->GetNewton(), width, height, gridsDiagonals, elevationdataType, elevationMap, attributeMap, vertcalScale, horizontalScale, 0));
	}

	CNEWTON_API dNewtonCollision* Clone (NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionHeightField (*this, shape);
	}

	protected:
	CNEWTON_API dNewtonCollisionHeightField (const dNewtonCollisionHeightField& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionNull: public dNewtonCollision
{
	public: 
	CNEWTON_API dNewtonCollisionNull (dNewton* const world)
		:dNewtonCollision(m_null, 0)
	{
		SetShape (NewtonCreateNull(world->GetNewton()));
	}

	CNEWTON_API dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionNull (*this, shape);
	}

	protected:
	CNEWTON_API dNewtonCollisionNull (const dNewtonCollisionNull& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionBox: public dNewtonCollision
{
	public: 
	CNEWTON_API dNewtonCollisionBox (dNewton* const world, dFloat x, dFloat y, dFloat z, dLong collisionMask)
		:dNewtonCollision(m_box, collisionMask)
	{
		SetShape (NewtonCreateBox(world->GetNewton(), x, y, z, 0, NULL));
	}

	CNEWTON_API dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionBox (*this, shape);
	}

	protected:
	CNEWTON_API dNewtonCollisionBox (const dNewtonCollisionBox& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionSphere: public dNewtonCollision
{
	public: 
	CNEWTON_API dNewtonCollisionSphere (dNewton* const world, dFloat radio, dLong collisionMask)
		:dNewtonCollision(m_sphere, collisionMask)
	{
		SetShape (NewtonCreateSphere(world->GetNewton(), radio, 0, NULL));
	}

	CNEWTON_API dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionSphere (*this, shape);
	}

	protected:
	CNEWTON_API dNewtonCollisionSphere (const dNewtonCollisionSphere& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionCapsule: public dNewtonCollision
{
	public: 
	CNEWTON_API dNewtonCollisionCapsule (NewtonCollision* const shape, dLong collisionMask)
		:dNewtonCollision(m_capsule, collisionMask)
	{
		SetShape (shape);
	}

	CNEWTON_API dNewtonCollisionCapsule (dNewton* const world, dFloat radio, dFloat height, dLong collisionMask)
		:dNewtonCollision(m_capsule, collisionMask)
	{
		SetShape (NewtonCreateCapsule (world->GetNewton(), radio, height, 0, NULL));
	}

	CNEWTON_API dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionCapsule (*this, shape);
	}

	protected:
	CNEWTON_API dNewtonCollisionCapsule (const dNewtonCollisionCapsule& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};



class dNewtonCollisionTaperedCapsule: public dNewtonCollision
{
	public: 
	CNEWTON_API dNewtonCollisionTaperedCapsule (dNewton* const world, dFloat radio0, dFloat radio1, dFloat height, dLong collisionMask)
		:dNewtonCollision(m_taperedCapsule, collisionMask)
	{
		SetShape (NewtonCreateTaperedCapsule (world->GetNewton(), radio0, radio1, height, 0, NULL));
	}

	CNEWTON_API dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionTaperedCapsule (*this, shape);
	}

	protected:
	CNEWTON_API dNewtonCollisionTaperedCapsule (const dNewtonCollisionTaperedCapsule& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionCone: public dNewtonCollision
{
	public: 
	CNEWTON_API dNewtonCollisionCone (dNewton* const world, dFloat radio, dFloat height, dLong collisionMask)
		:dNewtonCollision(m_cone, collisionMask)
	{
		SetShape (NewtonCreateCone (world->GetNewton(), radio, height, 0, NULL));
	}

	CNEWTON_API dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionCone (*this, shape);
	}

	protected:
	CNEWTON_API dNewtonCollisionCone (const dNewtonCollisionCone& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionCylinder: public dNewtonCollision
{
	public: 
	CNEWTON_API dNewtonCollisionCylinder (NewtonCollision* const shape, dLong collisionMask)
		:dNewtonCollision(m_cylinder, collisionMask)
	{
		SetShape (shape);
	}

	CNEWTON_API dNewtonCollisionCylinder (dNewton* const world, dFloat radio, dFloat height, dLong collisionMask)
		:dNewtonCollision(m_cylinder, collisionMask)
	{
		SetShape (NewtonCreateCylinder (world->GetNewton(), radio, height, 0, NULL));
	}

	CNEWTON_API dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionCylinder (*this, shape);
	}

	protected:
	CNEWTON_API dNewtonCollisionCylinder (const dNewtonCollisionCylinder& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionTaperedCylinder: public dNewtonCollision
{
	public: 
	CNEWTON_API dNewtonCollisionTaperedCylinder (dNewton* const world, dFloat radio0, dFloat radio1, dFloat height, dLong collisionMask)
		:dNewtonCollision(m_taperedCyinder, collisionMask)
	{
		SetShape (NewtonCreateTaperedCylinder(world->GetNewton(), radio0, radio1, height, 0, NULL));
	}

	CNEWTON_API dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionTaperedCylinder (*this, shape);
	}

	protected:
	CNEWTON_API dNewtonCollisionTaperedCylinder (const dNewtonCollisionTaperedCylinder& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionChamferedCylinder: public dNewtonCollision
{
	public: 
	CNEWTON_API dNewtonCollisionChamferedCylinder (dNewton* const world, dFloat radio, dFloat height, dLong collisionMask)
		:dNewtonCollision(m_chamferedCylinder, collisionMask)
	{
		SetShape (NewtonCreateChamferCylinder (world->GetNewton(), radio, height, 0, NULL));
	}

	CNEWTON_API dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionChamferedCylinder (*this, shape);
	}

	protected:
	CNEWTON_API dNewtonCollisionChamferedCylinder (const dNewtonCollisionChamferedCylinder& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionConvexHull: public dNewtonCollision
{
	public: 
	CNEWTON_API dNewtonCollisionConvexHull (NewtonCollision* const shape, dLong collisionMask)
		:dNewtonCollision(m_convex, collisionMask)
	{
		SetShape (shape);
	}

	CNEWTON_API dNewtonCollisionConvexHull (dNewton* const world, const dNewtonMesh& mesh, dLong collisionMask);

	CNEWTON_API dNewtonCollisionConvexHull (dNewton* const world, int vertexCount, const dFloat* const vertexCloud, int strideInBytes, dFloat tolerance, dLong collisionMask)
		:dNewtonCollision(m_convex, collisionMask)
	{
		SetShape (NewtonCreateConvexHull (world->GetNewton(), vertexCount, vertexCloud, strideInBytes, tolerance, 0, NULL));
	}

	CNEWTON_API dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionConvexHull (*this, shape);
	}

	protected:
	CNEWTON_API dNewtonCollisionConvexHull (const dNewtonCollisionConvexHull& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};


class dNewtonCollisionCompound: public dNewtonCollision
{
	public: 
	CNEWTON_API dNewtonCollisionCompound (NewtonCollision* const shape, dLong collisionMask)
		:dNewtonCollision(m_compound, collisionMask)
	{
		SetShape (shape);
	}

	CNEWTON_API dNewtonCollisionCompound (dNewton* const world, dLong collisionMask)
		:dNewtonCollision(m_compound, collisionMask)
	{
		SetShape (NewtonCreateCompoundCollision (world->GetNewton(), 0));
	}

	CNEWTON_API dNewtonCollisionCompound (dNewton* const world, const dNewtonMesh& mesh, dLong collisionMask);


	CNEWTON_API dNewtonCollision* Clone(NewtonCollision* const shape) const 
	{
		return new dNewtonCollisionCompound (*this, shape);
	}

	CNEWTON_API virtual void BeginAddRemoveCollision();
	CNEWTON_API virtual void* AddCollision(const dNewtonCollision* const collision);
	CNEWTON_API virtual void RemoveCollision (void* const handle);
	CNEWTON_API virtual void EndAddRemoveCollision();

	CNEWTON_API void* GetFirstNode () const;;
	CNEWTON_API void* GetNextNode (void* const collisionNode) const;
	CNEWTON_API dNewtonCollision* GetChildFromNode(void* const collisionNode) const; 

	protected:
	CNEWTON_API dNewtonCollisionCompound (const dNewtonCollisionCompound& srcCollision, NewtonCollision* const shape)
		:dNewtonCollision (srcCollision, shape)
	{
	}
};



#endif
