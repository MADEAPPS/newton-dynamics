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

#ifndef __ND_MODEL_NOTIFY_H__
#define __ND_MODEL_NOTIFY_H__

#include "ndModelStdafx.h"

class ndModelNotify: public ndBodyNotify
{
	public:
	//class ndModelNotifyFileLoadSave: public ndFileFormatNotify
	//{
	//	public:
	//	ndModelNotifyFileLoadSave(const char* const className = ndModelNotify::StaticClassName())
	//		:ndFileFormatNotify(className)
	//	{
	//	}
	//
	//	void SaveNotify(ndFileFormat* const scene, nd::TiXmlElement* const parentNode, const ndBodyNotify* const notify)
	//	{
	//		nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_NOTIFY_CLASS, ndModelNotify::StaticClassName());
	//		ndFileFormatNotify::SaveNotify(scene, classNode, notify);
	//		xmlSaveParam(classNode, "useCollisionForVisual", 1);
	//	}
	//};

	D_CLASS_REFLECTION(ndModelNotify, ndBodyNotify)
	ndModelNotify(ndBodyKinematic* const parentBody = nullptr, ndVector gravity = ndVector (ndFloat32 (0.0f), ndFloat32(-10.0f), ndFloat32(0.0f), ndFloat32(0.0f)));
	virtual ~ndModelNotify();

	virtual void OnTransform(ndInt32 threadIndex, const ndMatrix& matrix);
	virtual void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep);

	bool CheckInWorld(const ndMatrix& matrix) const;
	void CalculateMatrix(const ndMatrix& matrix, ndQuaternion& rot, ndVector& posit) const;

	ndBodyKinematic* m_parentBody;
};

#endif
