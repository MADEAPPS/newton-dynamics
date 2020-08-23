/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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
	DPLUGIN_API virtual ~dPluginCamera ();

	DPLUGIN_API virtual void SetPerspectiveMatrix(dSceneRender* const render, int width, int height);
	DPLUGIN_API virtual void SetOrtographicMatrix(dSceneRender* const render, int width, int height, const dMatrix& aligment);

	DPLUGIN_API virtual void DrawConstructionGrid(dSceneRender* const render) const;
	DPLUGIN_API virtual void BuildConstructionGrid(dSceneRender* const render, int count, dFloat spacing);
	DPLUGIN_API virtual void DestroyConstructionGrid(dSceneRender* const render);

	DPLUGIN_API virtual void DrawGizmo(dSceneRender* const render, int font) const;
	DPLUGIN_API virtual void DrawNodeSelectionGizmo(dSceneRender* const render, const dMatrix& matrix) const;
	DPLUGIN_API virtual void DrawNodeSelectAndMoveGizmo(dSceneRender* const render, const dMatrix& matrix) const;

	DPLUGIN_API virtual void SetZoom(dFloat zoom);
	DPLUGIN_API virtual dFloat GetZoom() const;

	DPLUGIN_API virtual void SetPanning(dFloat x, dFloat y);
	DPLUGIN_API virtual void GetPanning(dFloat & x, dFloat & y) const;

	DPLUGIN_API virtual void SetGridMatrix (const dMatrix& matrix);

	DPLUGIN_API virtual dFloat GetDistance() const;
	DPLUGIN_API virtual dFloat GetYawAngle() const;
	DPLUGIN_API virtual dFloat GetRollAngle() const;

	DPLUGIN_API virtual dVector GetPointOfInterest() const;
	DPLUGIN_API virtual void SetPointOfInterest(const dVector& pointOfInterst);

	DPLUGIN_API virtual dMatrix GetMatrix () const;
	DPLUGIN_API virtual void SetMatrix (dFloat yaw, dFloat roll, dFloat distance);

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