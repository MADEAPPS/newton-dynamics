/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "dgPhysicsStdafx.h"
#include "dgWorld.h"
#include "dgWorldPlugins.h"
/*
#include "dgDynamicBody.h"
#include "dgKinematicBody.h"
#include "dgCollisionBox.h"
#include "dgKinematicBody.h"
#include "dgCollisionNull.h"
#include "dgCollisionCone.h"
#include "dgCollisionScene.h"
#include "dgCollisionSphere.h"
#include "dgInverseDynamics.h"
#include "dgCollisionCapsule.h"
#include "dgBroadPhaseDefault.h"
#include "dgCollisionInstance.h"
#include "dgCollisionCompound.h"
#include "dgWorldDynamicUpdate.h"
#include "dgCollisionConvexHull.h"
#include "dgBroadPhasePersistent.h"
#include "dgCollisionChamferCylinder.h"
#include "dgUserConstraint.h"
#include "dgBallConstraint.h"
#include "dgHingeConstraint.h"
#include "dgSkeletonContainer.h"
#include "dgSlidingConstraint.h"
#include "dgUpVectorConstraint.h"
#include "dgUniversalConstraint.h"
#include "dgCorkscrewConstraint.h"
*/


dgPluginList::dgPluginList(dgMemoryAllocator* const allocator)
	:dgList<dgPluginModulePair>(allocator)
{
}

dgPluginList::~dgPluginList()
{
}

void dgPluginList::LoadPlugins()
{
	char plugInPath[2048];
	char rootPathInPath[2048];

#ifdef _MSC_VER
	GetModuleFileNameA(NULL, plugInPath, 256);
#endif

	for (dgInt32 i = strlen(plugInPath) - 1; i; i--) {
		if ((plugInPath[i] == '\\') || (plugInPath[i] == '/')) {
			plugInPath[i] = 0;
			break;
		}
	}
	strcat(plugInPath, "/newtonPlugins");
	sprintf(rootPathInPath, "%s/*.dll", plugInPath);


	dgPluginList& pluginsList = *this;

	// scan for all plugins in this folder
	_finddata_t data;
	intptr_t handle = _findfirst(rootPathInPath, &data);
	if (handle != -1) {
		do {
			sprintf(rootPathInPath, "%s/%s", plugInPath, data.name);
			HMODULE module = LoadLibrary(rootPathInPath);

			if (module) {
				// get the interface function pointer to the Plug in classes
				dgPlugin* const plugin = (dgPlugin*)GetProcAddress(module, "GetPlugin");

				if (plugin) {
					dgPluginModulePair entry(plugin, module);
					pluginsList.Append(entry);
				}
				else {
					FreeLibrary(module);
				}
			}

		} while (_findnext(handle, &data) == 0);

		_findclose(handle);
	}
}

void dgPluginList::UnloadPlugins()
{
	dgPluginList& pluginsList = *this;
	for (dgPluginList::dgListNode* node = pluginsList.GetFirst(); node; node = node->GetNext()) {
		HMODULE module = (HMODULE)node->GetInfo().m_module;
		FreeLibrary(module);
	}
}

dgPluginList::dgListNode* dgPluginList::GetCurrentPlugin()
{
	return NULL;
}


dgPluginList::dgListNode* dgPluginList::GetFirstPlugin()
{
	dgPluginList& list = *this;
	return list.GetFirst();
}

dgPluginList::dgListNode* dgPluginList::GetNextPlugin(dgListNode* const plugin)
{
	return plugin->GetNext();
}

const char* dgPluginList::GetPluginId(dgListNode* const plugin)
{
	dgPluginModulePair entry(plugin->GetInfo());
	//	void* xxxx = (void*) entry.m_plugin();
	return "xxxxx";
}

void dgPluginList::SelectPlugin(dgListNode* const plugin)
{
}
