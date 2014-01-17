/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#ifndef _DG_DEFORMABLE_BODY_UPDATE_H_
#define _DG_DEFORMABLE_BODY_UPDATE_H_



class dgCollisionDeformableMesh;

class dgDeformableBodiesUpdate: public dgList<dgCollisionDeformableMesh*>
{
	public:
	dgDeformableBodiesUpdate (dgMemoryAllocator* const allocator);

	void AddShape(dgCollisionDeformableMesh* const shape);
	void RemoveShape(dgCollisionDeformableMesh*  const shape);

	private:
    void ApplyExternaForces(dgFloat32 timestep);
    void SolveConstraintsAndIntegrate (dgFloat32 timestep);

	dgTree <dgListNode*, const dgCollisionDeformableMesh*> m_dictionary;
    friend class dgBroadPhase;
    friend class dgWorldDynamicUpdate;
};

#endif 

