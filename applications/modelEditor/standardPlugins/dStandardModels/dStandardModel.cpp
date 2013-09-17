/////////////////////////////////////////////////////////////////////////////
// Name:        dStandardModel.cpp
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
#include "dStandardModel.h"
#include "dBoxCollision.h"

// implement the UI for this using a dialog box
class dStandardModelUI : public FXDialogBox 
{
	public:
	dStandardModelUI()
	{
	}

	dStandardModelUI(FXWindow* const owner, dList<dCollisionPlugin*>& collisionsList)
		:FXDialogBox(owner, "Model properties", DECOR_TITLE|DECOR_BORDER, 0, 0, 300, 0) 
	{

		// add the type of available collision shape 
//		new FXHorizontalSeparator(this,LAYOUT_SIDE_TOP|LAYOUT_FILL_X|SEPARATOR_NONE);
//		new FXHorizontalSeparator(this,LAYOUT_SIDE_TOP|LAYOUT_FILL_X|SEPARATOR_NONE);
		new FXLabel(this, "Collision shape");
		FXComboBox* const collisionCombobox = new FXComboBox(this, collisionsList.GetCount(), NULL, 0, COMBOBOX_STATIC|FRAME_SUNKEN|FRAME_THICK|LAYOUT_SIDE_TOP);
		collisionCombobox->setNumVisible(collisionsList.GetCount());
		for (dList<dCollisionPlugin*>::dListNode* node = collisionsList.GetFirst(); node; node = node->GetNext()) {
			dCollisionPlugin* const collisionPlugin = (dCollisionPlugin*) node->GetInfo();
			collisionCombobox->appendItem(collisionPlugin->GetMenuName());
		}

		// add visual mesh type
		new FXHorizontalSeparator(this,LAYOUT_SIDE_TOP|LAYOUT_FILL_X|SEPARATOR_NONE);
		new FXLabel(this, "Visual mesh");
		FXComboBox* const visualMeshComboBox = new FXComboBox(this, collisionsList.GetCount(), NULL, 0, COMBOBOX_STATIC|FRAME_SUNKEN|FRAME_THICK|LAYOUT_SIDE_TOP);
		visualMeshComboBox->setNumVisible(1);
		visualMeshComboBox->appendItem("Set collision as visual mesh");

//		for (dList<dCollisionPlugin*>::dListNode* node = collisionsList.GetFirst(); node; node = node->GetNext()) {
			//dCollisionPlugin* const collisionPlugin = (dCollisionPlugin*) node->GetInfo();
			//visualMeshComboBox->appendItem(collisionPlugin->GetMenuName());
//		}


		// finish properties
		new FXHorizontalSeparator(this,LAYOUT_SIDE_TOP|LAYOUT_FILL_X|SEPARATOR_NONE);
		new FXHorizontalSeparator(this,LAYOUT_SIDE_TOP|LAYOUT_FILL_X|SEPARATOR_NONE);
		new FXHorizontalSeparator(this,LAYOUT_SIDE_TOP|LAYOUT_FILL_X|SEPARATOR_NONE);
		new FXHorizontalSeparator(this,LAYOUT_SIDE_TOP|LAYOUT_FILL_X|SEPARATOR_GROOVE);
		FXHorizontalFrame* const buttonContents = new FXHorizontalFrame(this,LAYOUT_SIDE_BOTTOM|FRAME_NONE|LAYOUT_FILL_X|PACK_UNIFORM_WIDTH);
		new FXButton(buttonContents,"&Accept",NULL,this,ID_ACCEPT, BUTTON_DEFAULT|BUTTON_INITIAL|FRAME_RAISED|FRAME_THICK|LAYOUT_CENTER_X|LAYOUT_CENTER_Y);
		new FXButton(buttonContents,"&Cancel",NULL,this,ID_CANCEL,BUTTON_DEFAULT|FRAME_RAISED|FRAME_THICK|LAYOUT_CENTER_X|LAYOUT_CENTER_Y);
	}

	~dStandardModelUI()
	{
	}

	FXDECLARE(dStandardModelUI)
};

FXIMPLEMENT(dStandardModelUI,FXDialogBox,NULL,0)


dStandardModel::dStandardModel()
	:dModelPlugin()
{
}

dStandardModel::~dStandardModel()
{
}


dStandardModel* dStandardModel::GetPlugin()
{
	static dStandardModel plugin;
	return &plugin;
}


bool dStandardModel::Create (dPluginInterface* const interface)
{
	// find all collision types
	dList<dBodyPlugin*> rigidBodies;
	dList<dCollisionPlugin*> collisionsList;
	
	for (void* node = interface->GetFirstPluginNode(); node; node = interface->GetNextPluginNode(node)) {
		dPluginRecord* const plugin = interface->GetPluginFromNode(node);

		switch (plugin->GetType())
		{
			case m_mesh:
//				collisionsList.Append((dPluginMesh*) plugin);
				break;
			case m_body:
				rigidBodies.Append((dBodyPlugin*) plugin);
				break;
			case m_collision:
				collisionsList.Append((dCollisionPlugin*) plugin);
				break;
		}
	}

	// get the relevant objects form the interface
	dPluginScene* const scene = interface->GetScene();
	NewtonWorld* const world = scene->GetNewtonWorld();
	FXMainWindow* const mainWindow = (FXMainWindow*) NewtonWorldGetUserData(world);
	dStandardModelUI modaldialog(mainWindow, collisionsList);

	if (modaldialog.execute(PLACEMENT_OWNER)) {
		interface->Push (new dUndoCurrentAsset(interface));
		//	dCollisionPlugin* const collisionPlugin = (dCollisionPlugin*) interface->GetPlugin("BoxCollision");
		//	_ASSERTE (collisionPlugin);

	}


	return true;
}


