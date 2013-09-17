/////////////////////////////////////////////////////////////////////////////
// Name:        dMeshBoxPrimitive.cpp
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 07:45:05
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

#include "StdAfx.h"
#include "dMeshBoxPrimitive.h"


// implement the UI for this using a dialog box
class dBoxPrimitiveUI : public FXDialogBox 
{
	public:
	dBoxPrimitiveUI()
	{
	}

	dBoxPrimitiveUI(FXWindow* const owner)
		:FXDialogBox(owner, "Box properties", DECOR_TITLE|DECOR_BORDER, 0, 0, 150, 0) 
	{

		new FXLabel(this, "name:");
		m_name = new FXTextField(this, 10, NULL, 0, TEXTFIELD_NORMAL|JUSTIFY_LEFT|LAYOUT_FILL_X|FRAME_SUNKEN|FRAME_THICK);
		m_name->setText("box primitive");

		new FXLabel(this, "dimensions:");
	    FXMatrix* const input = new FXMatrix(this, 2, FRAME_THICK|FRAME_RAISED|MATRIX_BY_COLUMNS|LAYOUT_FILL_X|LAYOUT_TOP|LAYOUT_LEFT,0,0,0,0,10,10,10,10);
		new FXLabel(input,"x:");
		m_x = new FXTextField(input, 10, NULL, 0, TEXTFIELD_REAL|JUSTIFY_RIGHT|FRAME_SUNKEN|FRAME_THICK);
		m_x->setText("1.0");

		new FXLabel(input,"y:");
		m_y = new FXTextField(input, 10, NULL, 0, TEXTFIELD_REAL|JUSTIFY_RIGHT|FRAME_SUNKEN|FRAME_THICK);
		m_y->setText("1.0");

		new FXLabel(input,"z:");
		m_z = new FXTextField(input, 10, NULL, 0, TEXTFIELD_REAL|JUSTIFY_RIGHT|FRAME_SUNKEN|FRAME_THICK);
		m_z->setText("1.0");

		// finish properties
		new FXHorizontalSeparator(this,LAYOUT_SIDE_TOP|LAYOUT_FILL_X|SEPARATOR_NONE);
		new FXHorizontalSeparator(this,LAYOUT_SIDE_TOP|LAYOUT_FILL_X|SEPARATOR_NONE);
		new FXHorizontalSeparator(this,LAYOUT_SIDE_TOP|LAYOUT_FILL_X|SEPARATOR_NONE);
		new FXHorizontalSeparator(this,LAYOUT_SIDE_TOP|LAYOUT_FILL_X|SEPARATOR_GROOVE);
		FXHorizontalFrame* const buttonContents = new FXHorizontalFrame(this,LAYOUT_SIDE_BOTTOM|FRAME_NONE|LAYOUT_FILL_X|PACK_UNIFORM_WIDTH);
		new FXButton(buttonContents,"&Accept",NULL,this,ID_ACCEPT, BUTTON_DEFAULT|BUTTON_INITIAL|FRAME_RAISED|FRAME_THICK|LAYOUT_CENTER_X|LAYOUT_CENTER_Y);
		new FXButton(buttonContents,"&Cancel",NULL,this,ID_CANCEL,BUTTON_DEFAULT|FRAME_RAISED|FRAME_THICK|LAYOUT_CENTER_X|LAYOUT_CENTER_Y);
	}

	~dBoxPrimitiveUI()
	{
	}

	
	FXTextField* m_x;
	FXTextField* m_y;
	FXTextField* m_z;
	FXTextField* m_name;

	FXDECLARE(dBoxPrimitiveUI)
};

FXIMPLEMENT(dBoxPrimitiveUI,FXDialogBox,NULL,0)


dMeshBoxPrimitive::dMeshBoxPrimitive()
	:dPluginMesh()
{
}

dMeshBoxPrimitive::~dMeshBoxPrimitive()
{
}


dMeshBoxPrimitive* dMeshBoxPrimitive::GetPlugin()
{
	static dMeshBoxPrimitive plugin;
	return &plugin;
}


dPluginScene* dMeshBoxPrimitive::Create (dPluginInterface* const interface)
{
	dPluginScene* const scene = interface->GetScene();
	NewtonWorld* const world = scene->GetNewtonWorld();
	FXMainWindow* const mainWindow = (FXMainWindow*) NewtonWorldGetUserData(world);
	dBoxPrimitiveUI modaldialog(mainWindow);

	dPluginScene* asset = NULL;
	if (modaldialog.execute(PLACEMENT_OWNER)) {
		// for now asset manage does not have undo/redo
		//interface->Push (new dUndoCurrentAsset(scene));

		asset = new dPluginScene(world);
		double x = atof (modaldialog.m_x->getText().text());
		double y = atof (modaldialog.m_y->getText().text());
		double z = atof (modaldialog.m_z->getText().text());

		dPluginScene::dTreeNode* const root = asset->GetRoot();
		dNodeInfo* const rootInfo = asset->GetInfoFromNode(root);
		rootInfo->SetName(modaldialog.m_name->getText().text());		


		dPluginScene::dTreeNode* const sceneNode = asset->CreateSceneNode(asset->GetRoot());
		dPluginScene::dTreeNode* const boxMesh = asset->CreateMeshNode(sceneNode);
		dMeshNodeInfo* const instance = (dMeshNodeInfo*) asset->GetInfoFromNode(boxMesh);
		instance->SetName("mesh");

		NewtonCollision* const boxCollision = NewtonCreateBox (world, float (x), float (y), float (z), 0, NULL); 
		NewtonMesh* const mesh = NewtonMeshCreateFromCollision(boxCollision);
		NewtonDestroyCollision(boxCollision);
		instance->ReplaceMesh (mesh);

//		char name[256];
//		sprintf (name, "%s_mesh", parent->GetName());
//		instance->SetName(name);
	}

	return asset;
}


void dMeshBoxPrimitive::Destroy (dPluginInterface* const interface, dPluginScene* const asset)
{
	asset->Release();
}
