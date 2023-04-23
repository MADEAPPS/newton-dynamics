/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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

class ndArchimedesBuoyancyVolume: public ndBodyTriggerVolume
{
	public:

	class ndFileDemoArchimedesBuoyancyVolume : public ndFileFormatBodyTriggerVolume
	{
		public:
		ndFileDemoArchimedesBuoyancyVolume()
			:ndFileFormatBodyTriggerVolume(ndArchimedesBuoyancyVolume::StaticClassName())
		{
		}

		void SaveBody(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndBody* const body)
		{
			nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_BODY_CLASS, ndArchimedesBuoyancyVolume::StaticClassName());
			ndFileFormatBodyTriggerVolume::SaveBody(scene, classNode, body);

			ndArchimedesBuoyancyVolume* const buoyancy = (ndArchimedesBuoyancyVolume*)body;
			xmlSaveParam(classNode, "planeNormal", buoyancy->m_plane);
			xmlSaveParam(classNode, "planeDistance", -buoyancy->m_plane.m_w);
			xmlSaveParam(classNode, "density", buoyancy->m_density);
			xmlSaveParam(classNode, "hasPlane", buoyancy->m_hasPlane ? 1 : 0);
		}
	};

	D_CLASS_REFLECTION(ndArchimedesBuoyancyVolume, ndBodyTriggerVolume)
	ndArchimedesBuoyancyVolume();

	void CalculatePlane(ndBodyKinematic* const body);
	void OnTriggerEnter(ndBodyKinematic* const body, ndFloat32 timestep);
	void OnTrigger(ndBodyKinematic* const kinBody, ndFloat32 timestep);
	void OnTriggerExit(ndBodyKinematic* const body, ndFloat32 timestep);

	ndPlane m_plane;
	ndFloat32 m_density;
	bool m_hasPlane;
};

#endif