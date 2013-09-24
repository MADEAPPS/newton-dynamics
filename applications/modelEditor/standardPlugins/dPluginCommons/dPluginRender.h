/////////////////////////////////////////////////////////////////////////////
// Name:        dSceneNodeInfo.h
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 08:02:08
// RCS-ID:      
// Copyright:   Copyright (c) <2010> <Newton Game Dynamics>
// License:     
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
// 
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely
/////////////////////////////////////////////////////////////////////////////

#ifndef _D_PLUGIN_RENDER_H_
#define _D_PLUGIN_RENDER_H_

#include "dPluginUtils.h"

class dPluginRender: public dSceneRender
{
	public:
	dPluginRender(void);
	virtual ~dPluginRender(void);

	virtual bool Init();

	virtual int GetViewPortWidth() const;
	virtual int GetViewPortHeight() const;

	virtual dMatrix GetProjectionMatrix () const;
	virtual dMatrix GetModelViewMatrix() const; 

	virtual void SetModelViewMatrix(const dMatrix& modelview); 
	virtual void SetProjectionMatrix (const dMatrix& projection);

	virtual void SetOrtographicProjection (int width, int height, dFloat minPlane, dFloat maxPlane);
	virtual void SetPerspectiveProjection (int width, int height, dFloat fov, dFloat frontPlane, dFloat backPlane);

	virtual void Print (int displayListFont, dFloat x, dFloat y, const char* const fmt, ... );

	virtual void LoadMatrix(const dMatrix& matrix);
	virtual void PushMatrix(const dMatrix& matrix);
	virtual void PopMatrix();

	virtual void BeginRender();
	virtual void EndRender();

	virtual int CreateDisplayList(int range = 1);
	virtual void DestroyDisplayList(int lists, int range = 1);

	virtual void BeginDisplayList(int displayList);
	virtual void EndDisplayList();
	virtual void DrawDisplayList(int displayList);

	virtual int GetCachedWireframeDisplayList(NewtonMesh* const mesh); 
	virtual int GetCachedFlatShadedDisplayList(NewtonMesh* const mesh); 
	

	// material interface
	virtual void EnableZbuffer();
	virtual void DisableZbuffer();
	virtual void EnableBackFace();
	virtual void DisableBackFace();
	virtual void EnableBlend();
	virtual void DisableBlend();
	virtual void EnableLighting();
	virtual void DisableLighting();
	virtual void EnableTexture();
	virtual void DisableTexture();
	virtual void EnableZBias(dFloat val);
	virtual void DisableZBias();
	virtual void SetColor(const dVector& color);
	virtual void SetMaterialDiffuse(const dVector& color);
	virtual void SetMaterialAmbient(const dVector& color);
	virtual void SetMaterialSpecular(const dVector& color);
	virtual void SetMaterialShininess(dFloat normalizedPower);

	
	// primitive drawing functions
	virtual void BeginLine();
	virtual void BeginTriangle();
	virtual void SubmitNormal(const dVector& normal);
	virtual void SubmitVertex(const dVector& posit);
	virtual void End();


	
	private:
	void CleanupDiaplayListCache(dTree<int, NewtonMesh*>& cache);
	dTree<int, NewtonMesh*> m_wireFrameDisplayList;
	dTree<int, NewtonMesh*> m_flatShadedDisplayList;
};


#endif