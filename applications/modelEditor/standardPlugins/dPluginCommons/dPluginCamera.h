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

// NewtonModelEditor.cpp : Defines the entry point for the application.
//

#ifndef _D_PLUGIN_CAMEREA_H_
#define _D_PLUGIN_CAMEREA_H_

#include "dPluginUtils.h"
 
class dPluginCamera
{
	public:
	DPLUGIN_API dPluginCamera ();
	virtual DPLUGIN_API ~dPluginCamera ();

	virtual DPLUGIN_API void SetPerspectiveMatrix(dSceneRender* const render, int width, int height);
	virtual DPLUGIN_API void SetOrtographicMatrix(dSceneRender* const render, int width, int height, const dMatrix& aligment);

	virtual DPLUGIN_API void DrawConstructionGrid(dSceneRender* const render) const;
	virtual DPLUGIN_API void BuildConstructionGrid(dSceneRender* const render, int count, dFloat spacing);
	virtual DPLUGIN_API void DestroyConstructionGrid(dSceneRender* const render);

	virtual DPLUGIN_API void DrawGizmo(dSceneRender* const render, int font) const;
	virtual DPLUGIN_API void DrawNodeSelectionGizmo(dSceneRender* const render, const dMatrix& matrix) const;
	virtual DPLUGIN_API void DrawNodeSelectAndMoveGizmo(dSceneRender* const render, const dMatrix& matrix) const;

	virtual DPLUGIN_API void SetZoom(dFloat zoom);
	virtual DPLUGIN_API dFloat GetZoom() const;

	virtual DPLUGIN_API void SetPanning(dFloat x, dFloat y);
	virtual DPLUGIN_API void GetPanning(dFloat & x, dFloat & y) const;

	virtual DPLUGIN_API void SetGridMatrix (const dMatrix& matrix);

	virtual DPLUGIN_API dFloat GetDistance() const;
	virtual DPLUGIN_API dFloat GetYawAngle() const;
	virtual DPLUGIN_API dFloat GetRollAngle() const;

	virtual DPLUGIN_API dVector GetPointOfInterest() const;
	virtual DPLUGIN_API void SetPointOfInterest(const dVector& pointOfInterst);

	virtual DPLUGIN_API dMatrix GetMatrix () const;
	virtual DPLUGIN_API void SetMatrix (dFloat yaw, dFloat roll, dFloat distance);

	private:
	dMatrix m_matrix;
	dMatrix m_gridAligment;
	dVector m_pointOfInterest;
	dFloat m_distance;
	dFloat m_panX;
	dFloat m_panY;
	dFloat m_zoom;
	dFloat m_fov;
	dFloat m_backPlane;
	dFloat m_frontPlane;
	float m_cameraYaw;
	float m_cameraRoll;
	unsigned m_grid;
};



#endif