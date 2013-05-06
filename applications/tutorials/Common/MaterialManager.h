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

#ifndef __MATERIAL_MANAGER__H
#define __MATERIAL_MANAGER__H

struct PhysicsMaterialInteration
{
	dFloat m_restitution;
	dFloat m_staticFriction;
	dFloat m_kineticFriction;

	void* m_impactSound;
	void* m_scrapingSound;
};

class SoundManager;

// Create an advance Material Manager 
void* CreateMaterialManager (NewtonWorld* world, SoundManager* soudnManager);

// Set Default Material
void AddDefaultMaterial (void* manager, PhysicsMaterialInteration* interation);

// This Add and Interaction between two Material to the database 
void AddMaterilInteraction (void* manager, int MaterialId_0, int MaterialId_1, PhysicsMaterialInteration* interation);

#endif
