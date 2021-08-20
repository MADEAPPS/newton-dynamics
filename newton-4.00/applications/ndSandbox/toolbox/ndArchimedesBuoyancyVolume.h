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

#ifndef D_ARCHIMIDES_BOUYANCY_VOLUME_H
#define D_ARCHIMIDES_BOUYANCY_VOLUME_H

#include "ndSandboxStdafx.h"
#include "ndPhysicsUtils.h"
#include "ndDemoEntityManager.h"

class ndArchimedesBuoyancyVolume : public ndBodyTriggerVolume
{
	public:
	ndArchimedesBuoyancyVolume();
	ndArchimedesBuoyancyVolume(const dLoadSaveBase::dLoadDescriptor& desc);

	void CalculatePlane(ndBodyKinematic* const body);
	void OnTriggerEnter(ndBodyKinematic* const body, dFloat32 timestep);
	void OnTrigger(ndBodyKinematic* const kinBody, dFloat32 timestep);
	void OnTriggerExit(ndBodyKinematic* const body, dFloat32 timestep);
	virtual void Save(nd::TiXmlElement* const rootNode, const char* const assetPath, dInt32 shapeHash, dInt32 nodeHash) const;

	dPlane m_plane;
	dFloat32 m_density;
	bool m_hasPlane;
};

#endif