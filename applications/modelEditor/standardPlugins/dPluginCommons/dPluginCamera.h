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


class dPluginCamera
{
	public:
	dPluginCamera ();
	virtual ~dPluginCamera ();

	virtual void SetPerspectiveMatrix(dSceneRender* const render, int width, int height);
	virtual void SetOrtographicMatrix(dSceneRender* const render, int width, int height, const dMatrix& aligment);

	virtual void DrawConstructionGrid(dSceneRender* const render) const;
	virtual void BuildConstructionGrid(dSceneRender* const render, int count, dFloat spacing);
	virtual void DestroyConstructionGrid(dSceneRender* const render);

	virtual void DrawGizmo(dSceneRender* const render, int font) const;
	virtual void DrawNodeSelectionGizmo(dSceneRender* const render, const dMatrix& matrix) const;
	virtual void DrawNodeSelectAndMoveGizmo(dSceneRender* const render, const dMatrix& matrix) const;


	virtual void SetZoom(dFloat zoom);
	virtual dFloat GetZoom() const;

	virtual void SetPanning(dFloat x, dFloat y);
	virtual void GetPanning(dFloat & x, dFloat & y) const;

	virtual void SetGridMatrix (const dMatrix& matrix);

	virtual dFloat GetDistance() const;
	virtual dFloat GetYawAngle() const;
	virtual dFloat GetRollAngle() const;

	virtual dVector GetPointOfInterest() const;
	virtual void SetPointOfInterest(const dVector& pointOfInterst);

	virtual dMatrix GetMatrix () const;
	virtual void SetMatrix (dFloat yaw, dFloat roll, dFloat distance);
	

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