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


#include "dPluginStdafx.h"
#include "dPluginCamera.h"

#define D_ORTOGONAL_PIXEL_TO_METERS_SCALE	(40.0f)
#define D_PERSPECTIVE_PIXEL_TO_METERS_SCALE	(2.0f)
#define D_PANNING_SENSITIVITY				(1.0f / 32.0f)


dPluginCamera::dPluginCamera()
	:m_matrix (GetIdentityMatrix()) 
	,m_gridAligment (GetIdentityMatrix())
	,m_pointOfInterest(0.0f, 0.0f, 0.0f, 0.0f)
	,m_distance(15.0f)
	,m_panX(0.0f)
	,m_panY(0.0f)
	,m_zoom(1.0f)
	,m_fov (60.0f * 3.1416f / 180.0f)
	,m_frontPlane (0.1f)
	,m_backPlane(2000.0f)
	,m_cameraYaw(45.0f * 3.1416f/180.0f)
	,m_cameraRoll(-30.0f * 3.1416f/180.0f)
	,m_grid(0)
{
	SetMatrix(m_cameraYaw, m_cameraRoll, m_distance);
}


dPluginCamera::~dPluginCamera()
{
	_ASSERTE (!m_grid);
}


void dPluginCamera::SetPerspectiveMatrix(dSceneRender* const render, int width, int height)
{
	// set the perspective matrix
	render->SetPerspectiveProjection(width, height, m_fov * 180.0f / 3.1416f, m_frontPlane, m_backPlane);

	// calculate the same gluLookAt matrix
	dMatrix modelViewMatrix(GetIdentityMatrix());
	modelViewMatrix[2] = m_matrix.m_front.Scale (-1.0f);
	modelViewMatrix[0] = m_matrix.m_up * modelViewMatrix[2];
	modelViewMatrix[1] = modelViewMatrix[2] * modelViewMatrix[0];
	modelViewMatrix[3] = m_matrix.m_posit;
	modelViewMatrix = modelViewMatrix.Inverse();

	// apply scale, zoom and pan 
	dMatrix zoomMatrix(GetIdentityMatrix());
	zoomMatrix[0][0] = D_PERSPECTIVE_PIXEL_TO_METERS_SCALE * m_zoom;
	zoomMatrix[1][1] = D_PERSPECTIVE_PIXEL_TO_METERS_SCALE * m_zoom;
	zoomMatrix[2][2] = D_PERSPECTIVE_PIXEL_TO_METERS_SCALE * m_zoom;

	dMatrix panMatrix (GetIdentityMatrix());
	// use a pan sensitivity 0f 0.25f
	dFloat panSensitivity = D_PANNING_SENSITIVITY;
	panMatrix.m_posit = dVector(m_panX * panSensitivity, m_panY * panSensitivity, 0.0f, 1.0f);

	dMatrix matrix (zoomMatrix * modelViewMatrix * panMatrix);
	render->SetModelViewMatrix(matrix);
}


void dPluginCamera::SetOrtographicMatrix(dSceneRender* const render, int width, int height, const dMatrix& aligment)
{
	render->SetOrtographicProjection (width, height, -m_backPlane, m_backPlane);

	// set the model view matrix 
	dMatrix zoomMatrix(GetIdentityMatrix());

	// us an arbitrary pixels to meters scale = 5.0f
	zoomMatrix[0][0] = D_ORTOGONAL_PIXEL_TO_METERS_SCALE * m_zoom;
	zoomMatrix[1][1] = D_ORTOGONAL_PIXEL_TO_METERS_SCALE * m_zoom;
	zoomMatrix[2][2] = D_ORTOGONAL_PIXEL_TO_METERS_SCALE * m_zoom;

	dMatrix panMatrix (GetIdentityMatrix());
	panMatrix.m_posit = dVector(width/2.0f + m_panX, height/2.0f + m_panY, 0.0f, 1.0f);

	dMatrix matrix (zoomMatrix * aligment * panMatrix);
	render->SetModelViewMatrix(matrix);
}


dFloat dPluginCamera::GetYawAngle() const
{
	return m_cameraYaw;
}

dFloat dPluginCamera::GetRollAngle() const
{
	return m_cameraRoll;
}

dFloat dPluginCamera::GetDistance() const
{
	return m_distance;
}

dMatrix dPluginCamera::GetMatrix () const
{
	return m_matrix;
}

dVector dPluginCamera::GetPointOfInterest() const
{
	return m_pointOfInterest;
}

void dPluginCamera::SetPointOfInterest(const dVector& pointOfInterest)
{
	_ASSERTE (0);
	m_pointOfInterest = pointOfInterest;
}



void dPluginCamera::SetMatrix (dFloat yaw, dFloat roll, dFloat distance)
{
//	dMatrix yawMatrix_ (dYawMatrix(yaw));
//	dMatrix rollMatrix_ (dRollMatrix(roll));
//	m_matrix = rollMatrix_ * yawMatrix_;
//	m_matrix.m_posit = dVector (-10.0f, 5.0f, 10.0f, 1.0f);

	m_cameraRoll = dClamp(roll, -85.0f * 3.141692f / 180.0f, 85.0f * 3.141692f / 180.0f);
	m_cameraYaw = dMod (yaw, 2.0f * 3.141592f);
	m_distance = (distance < 0.5f) ? 0.5f : distance;

	dMatrix yawMatrix (dYawMatrix(m_cameraYaw));
	dMatrix rollMatrix (dRollMatrix(m_cameraRoll));
	m_matrix = rollMatrix * yawMatrix;
	m_matrix.m_posit = m_matrix.RotateVector(dVector (-distance, 0.0f, 0.0f, 1.0f)) + m_pointOfInterest;
	m_matrix.m_posit.m_w = 1.0f;
}



void dPluginCamera::SetZoom(dFloat zoom)
{
	m_zoom = zoom;
}

dFloat dPluginCamera::GetZoom() const
{
	return m_zoom;
}


void dPluginCamera::SetPanning(dFloat x, dFloat y)
{
	m_panX = x;
	m_panY = y;
}

void dPluginCamera::GetPanning(dFloat& x, dFloat& y) const
{
	x = m_panX;
	y = m_panY;
}


void dPluginCamera::BuildConstructionGrid(dSceneRender* const render, int count, dFloat spacing)
{
	_ASSERTE (!m_grid);

	render->PushMatrix(GetIdentityMatrix());
	render->LoadMatrix(GetIdentityMatrix());

	m_grid = render->CreateDisplayList();
	render->BeginDisplayList(m_grid);

	render->DisableZbuffer();
	render->DisableBackFace();
	render->DisableLighting ();
	render->DisableTexture();
	render->DisableBlend();

	render->BeginLine();
	count = (count & ~1) + 1;

	dFloat y = - (count / 2) * spacing;
	render->SetColor(dVector (0.4f, 0.4f, 0.4f, 0.0f));
	for (int j = 0; j < count; j ++) {
		dFloat val = spacing * (count / 2);
		render->DrawLine (dVector(- val,     y, 0.0f, 0.0f), dVector(  val,   y, 0.0f, 0.0f));
		render->DrawLine (dVector(    y, - val, 0.0f, 0.0f), dVector(    y, val, 0.0f, 0.0f));
		y += spacing;
	}

	render->SetColor(dVector (0.0f, 0.0f, 0.0f, 0.0f));
	render->DrawLine (dVector(- spacing * (count / 2), 0.0f, 0.0f, 0.0f), dVector(  spacing * (count / 2), 0.0f, 0.0f, 0.0f));
	render->DrawLine (dVector(0.0f, - spacing * (count / 2), 0.0f, 0.0f), dVector(0.0f,   spacing * (count / 2), 0.0f, 0.0f));

	render->End();
	render->EndDisplayList();
	render->PopMatrix();
}


void dPluginCamera::DestroyConstructionGrid(dSceneRender* const render)
{
	render->DestroyDisplayList(m_grid);
	m_grid = 0;
}

void dPluginCamera::SetGridMatrix (const dMatrix& matrix)
{
	m_gridAligment = matrix;
}


void dPluginCamera::DrawConstructionGrid(dSceneRender* const render) const
{
	render->PushMatrix(m_gridAligment);
	render->DrawDisplayList(m_grid);
	render->PopMatrix();
}


void dPluginCamera::DrawGizmo(dSceneRender* const render, int font) const
{
	render->DisableZbuffer();
	//render->EnableZbuffer();
	render->EnableBackFace();
	render->DisableLighting ();
	render->DisableTexture();
	render->DisableBlend();

	// calculate  gizmo size
	dFloat zbuffer = 0.5f;


	// calculate a point the lower left corner of the screen at the front plane in global space 
	dFloat x0 = 40.0f;
	dFloat y0 = render->GetViewPortHeight() - 30.0f;
	dVector origin (render->ScreenToGlobal(dVector (x0, y0, zbuffer, 1.0f)));

	dFloat length = 30.0f;
	dVector p1 (render->ScreenToGlobal(dVector (x0 + length, y0, zbuffer, 1.0f)));
	dVector p1p0(p1 - origin);
	length = dSqrt (p1p0 % p1p0);
	
	// display x axis
	{
		dMatrix matrix (GetIdentityMatrix());
		matrix.m_posit = origin;
		render->PushMatrix(matrix);
		render->BeginLine();
		render->SetColor(dVector (1.0f, 0.0f, 0.0f, 0.0f));
		render->DrawLine (dVector (0.0f, 0.0f, 0.0f, 0.0f), dVector (length, 0.0f, 0.0f, 0.0f));
		render->End();
		dVector posit (render->GlobalToScreen(dVector (length * 1.2f, 0.0f, 0.0f, 0.0f)));
		render->SetColor(dVector (1.0f, 1.0f, 1.0f, 0.0f));
		render->Print(font, posit.m_x, posit.m_y, "x");
		render->PopMatrix();
	}

	// display y axis
	{
		dMatrix matrix (dRollMatrix((90.0f * 3.141592f / 180.0f)));
		matrix.m_posit = origin;
		render->PushMatrix(matrix);
		render->BeginLine();
		render->SetColor(dVector (0.0f, 1.0f, 0.0f, 0.0f));
		render->DrawLine (dVector (0.0f, 0.0f, 0.0f, 0.0f), dVector (length, 0.0f, 0.0f, 0.0f));
		render->End();
		dVector posit (render->GlobalToScreen(dVector (length * 1.2f, 0.0f, 0.0f, 0.0f)));
		render->SetColor(dVector (1.0f, 1.0f, 1.0f, 0.0f));
		render->Print(font, posit.m_x, posit.m_y, "y");
		render->PopMatrix();
	}

	// display z axis
	{
		dMatrix matrix (dYawMatrix((-90.0f * 3.141592f / 180.0f)));
		matrix.m_posit = origin;
		render->PushMatrix(matrix);
		render->BeginLine();
		render->SetColor(dVector (0.0f, 0.0f, 1.0f, 0.0f));
		render->DrawLine (dVector (0.0f, 0.0f, 0.0f, 0.0f), dVector (length, 0.0f, 0.0f, 0.0f));
		render->End();
		dVector posit (render->GlobalToScreen(dVector (length * 1.2f, 0.0f, 0.0f, 0.0f)));
		render->SetColor(dVector (1.0f, 1.0f, 1.0f, 0.0f));
		render->Print(font, posit.m_x, posit.m_y, "z");
		render->PopMatrix();
	}
}

void dPluginCamera::DrawNodeSelectionGizmo(dSceneRender* const render, const dMatrix& matrix) const
{
	render->DisableZbuffer();
	render->EnableBackFace();
	render->DisableLighting ();
	render->DisableTexture();
	render->DisableBlend();


	render->PushMatrix(matrix);

	render->BeginLine();
	render->SetColor(dVector (1.0f, 0.0f, 0.0f, 0.0f));
	render->DrawLine (dVector (0.0f, 0.0f, 0.0f, 0.0f), dVector (1.0f, 0.0f, 0.0f, 0.0f));

	render->SetColor(dVector (0.0f, 1.0f, 0.0f, 0.0f));
	render->DrawLine (dVector (0.0f, 0.0f, 0.0f, 0.0f), dVector (0.0f, 1.0f, 0.0f, 0.0f));

	render->SetColor(dVector (0.0f, 0.0f, 1.0f, 0.0f));
	render->DrawLine (dVector (0.0f, 0.0f, 0.0f, 0.0f), dVector (0.0f, 0.0f, 1.0f, 0.0f));

	render->End();

	render->PopMatrix();
}



void dPluginCamera::DrawNodeSelectAndMoveGizmo(dSceneRender* const render, const dMatrix& matrix) const
{
	render->DisableZbuffer();
	render->EnableBackFace();
	//render->DisableLighting ();
	render->EnableLighting ();
	render->DisableTexture();
	render->DisableBlend();


	
	//lightPosition.m_w = 0.0f;
	//const GLfloat lightPosition[]={-1.0f, 0.0f, 0.0f, 0.0f};

	
	const GLfloat lightAmbient[]={0.0f,0.0f,0.0f,1.0f};
	const GLfloat lightDiffuse[]={1.0f,1.0f,1.0f,1.0f};

	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
	glEnable(GL_LIGHT0);



	render->SetMaterialAmbient(dVector (0.0f, 0.0f, 0.0f, 0.0f));
	render->SetMaterialSpecular(dVector (0.0f, 0.0f, 0.0f, 0.0f));
	render->SetMaterialShininess(0.0f);

	//glLightfv(GL_LIGHT0, GL_POSITION, &m_matrix.m_posit[0]);
dVector xxx (m_matrix[0].Scale (-1.0f));
	glLightfv(GL_LIGHT0, GL_POSITION, &xxx[0]);

	glEnable(GL_LIGHT0);
	render->EnableLighting ();



	render->SetColor(dVector (1.0f, 1.0f, 1.0f, 0.0f));

	int segments = 8;
	dFloat radio = 0.125f * 4;
	dFloat height = 0.25f * 4;
	{
		render->SetMaterialDiffuse(dVector (1.0f, 0.0f, 0.0f, 1.0f));
		dMatrix tranlation (GetIdentityMatrix());
		tranlation.m_posit.m_x = 1.0f + height / 2;
		render->PushMatrix(tranlation * matrix);

/*
dFloat x0 = render->GetViewPortWidth() / 2.0f;
dFloat y0 = render->GetViewPortHeight() / 2.0f;
dVector p0 (render->ScreenToGlobal (dVector (x0, y0, 0.0f, 0.0f)));
dVector p1 (render->ScreenToGlobal (dVector (x0, y0, 1.0f, 0.0f)));
dVector dir (p0 - p1);
dMatrix matrix1(render->GetModelViewMatrix().Inverse4x4());
dVector dir1 (matrix1.RotateVector4x4(dir));
dir1.m_w = 0.0f;
glLightfv(GL_LIGHT0, GL_POSITION, &dir1[0]);

//		dMatrix matrix(render->GetModelViewMatrix().Inverse4x4());
//		dVector lightPosition  (matrix.RotateVector4x4(dVector(-1.0f, 0.0f, 0.0f, 0.0f)));
//		lightPosition.m_w = 0.0f;
//		glLightfv(GL_LIGHT0, GL_POSITION, &lightPosition[0]);
*/
dVector lightPosition1;
glGetLightfv (GL_LIGHT0, GL_POSITION, &lightPosition1[0]);



		render->DrawCone (segments, radio, height);
		render->PopMatrix();
	}

	
	DrawNodeSelectionGizmo(render, matrix);
}
