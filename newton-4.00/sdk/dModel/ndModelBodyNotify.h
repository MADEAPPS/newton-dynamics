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

#ifndef __ND_MODEL_BODY_NOTIFY_H__
#define __ND_MODEL_BODY_NOTIFY_H__

#include "ndModelStdafx.h"

class ndModelBodyNotify: public ndBodyNotify
{
	public:
	//class ndModelBodyNotifyFileLoadSave: public ndFileFormatNotify
	//{
	//	public:
	//	ndModelBodyNotifyFileLoadSave(const char* const className = ndModelBodyNotify::StaticClassName())
	//		:ndFileFormatNotify(className)
	//	{
	//	}
	//
	//	void SaveNotify(ndFileFormat* const scene, nd::TiXmlElement* const parentNode, const ndBodyNotify* const notify)
	//	{
	//		nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_NOTIFY_CLASS, ndModelBodyNotify::StaticClassName());
	//		ndFileFormatNotify::SaveNotify(scene, classNode, notify);
	//		xmlSaveParam(classNode, "useCollisionForVisual", 1);
	//	}
	//};

	D_CLASS_REFLECTION(ndModelBodyNotify, ndBodyNotify)

	ndModelBodyNotify(const ndModelBodyNotify& src);
	ndModelBodyNotify(ndBodyKinematic* const parentBody = nullptr, ndVector gravity = ndVector (ndFloat32 (0.0f), ndFloat32(-10.0f), ndFloat32(0.0f), ndFloat32(0.0f)));
	virtual ~ndModelBodyNotify();

	ndBodyNotify* Clone() const
	{
		return new ndModelBodyNotify(*this);
	}

	virtual void OnTransform(ndInt32 threadIndex, const ndMatrix& matrix);
	virtual void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep);

	bool CheckInWorld(const ndMatrix& matrix) const;
	void CalculateMatrix(const ndMatrix& matrix, ndQuaternion& rot, ndVector& posit) const;

	ndBodyKinematic* m_parentBody;
};

#endif
