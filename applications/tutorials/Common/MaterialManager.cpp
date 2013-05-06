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

#include "StdAfx.h"
#include "dVector.h"
#include "SoundManager.h"
#include "MaterialManager.h"
#include "CollectInputAndUpdateCamera.h"

#define MAX_HASH_COLLISION	2

static unsigned randBits[] = 
{
	0x00000001, 0x2C11F801, 0xDFD8F60E, 0x6C8FA2B7, 
	0xB573754C, 0x1522DCDD, 0x21615D3A, 0xE1B307F3, 
	0x12AFA158, 0x53D18179, 0x70950126, 0x941702EF, 
	0x756FE824, 0x694801D5, 0x36DF4DD2, 0x63D80FAB, 
	0xB8AE95B0, 0x902439F1, 0x090C6F3E, 0x2B7C6A27, 
	0x8344B5FC, 0x67D3C5CD, 0x22F5516A, 0x2FB00E63, 
	0xFC761508, 0x87B00169, 0x27EBA056, 0x8CC0B85F, 
	0xE33D3ED4, 0x95DA08C5, 0x13E5C802, 0x9DD9E41B, 
	0xD4577F60, 0x3DD6B7E1, 0x096AF46E, 0x1A00CD97, 
	0x4B10E2AC, 0x22EAAABD, 0x683F119A, 0x62D070D3, 
	0xA8D034B8, 0xAA363D59, 0x58CECB86, 0x40F589CF, 
	0x4F630184, 0x38918BB5, 0xB85B8E32, 0x0A6A948B, 
	0x9A099510, 0x402871D1, 0x11E7859E, 0xEE73CD07, 
	0x4142FB5C, 0x39D68BAD, 0x0FE19DCA, 0xE35B2F43, 
	0x75590068, 0x66433549, 0x929182B6, 0x71EC773F, 
	0xBBAC3034, 0xF2BD8AA5, 0x5743A062, 0x5AB120FB, 
	0x5ABFD6C0, 0xDDD867C1, 0xDC3522CE, 0xD0EC6877, 
	0xE106000C, 0xB7C6689D, 0xED3FF5FA, 0xC75749B3, 
	0x126B7818, 0x1A75E939, 0x0546C5E6, 0x8A9C80AF, 
	0x48A3CAE4, 0x756D0595, 0x7060FE92, 0xA594896B, 
	0x12354470, 0x896599B1, 0xDAC6CBFE, 0xCB419FE7, 
	0x9C44F0BC, 0xAFA9418D, 0xB87D1A2A, 0x428BC023, 
	0x33229BC8, 0xC92D5929, 0xB1C19516, 0x0FBCA61F, 
	0xE594D194, 0x716EFC85, 0x0036A8C2, 0xD7BBCDDB, 
	0x16E4DE20, 0xD10F07A1, 0x68CF812E, 0x390A7357, 
	0x8BAACD6C, 0x2C2E167D, 0x3E7C0A5A, 0x167F9293, 
	0x3D596B78, 0x08888519, 0x9994F046, 0x0FC3E78F, 
	0x008A4444, 0x87526F75, 0xB0079EF2, 0x238DEE4B, 
	0xCA09A3D0, 0x4ED3B191, 0xFA42425E, 0x379DE2C7, 
	0x1EA2961C, 0x1FC3E76D, 0x90DFC68A, 0x0279C103, 
	0xF9AAE728, 0xF2666D09, 0xEF13D776, 0x92E944FF, 
	0x364F22F4, 0x37665E65, 0x05D6E122, 0x7131EABB, 
	0x479E9580, 0x98729781, 0x4BD20F8E, 0x1612EE37, 
	0xCB574ACC, 0x5499B45D, 0x360B4EBA, 0x33814B73, 
	0x43720ED8, 0x146610F9, 0x45514AA6, 0x0B23BE6F, 
	0x026E6DA4, 0xD1B9C955, 0x94676F52, 0xCE8EC32B, 
	0x165EB330, 0x2F6AB971, 0x92F1E8BE, 0xC54095A7, 
	0xBEB3EB7C, 0x5C9E7D4D, 0x5921A2EA, 0xB45D31E3, 
	0xB8C9E288, 0x5FE670E9, 0xC02049D6, 0xC42A53DF, 
	0x6F332454, 0x661BB045, 0x2B3C4982, 0xDF4B779B, 
	0xD7C4FCE0, 0x70FB1761, 0xADD4CDEE, 0x47BDD917, 
	0x8C63782C, 0x8181423D, 0xFA05C31A, 0xDD947453, 
	0x6A8D6238, 0x1A068CD9, 0x4413D506, 0x5374054F, 
	0xC5A84704, 0xB41B1335, 0x06986FB2, 0x4CCF080B, 
	0xF80C7290, 0x8622B151, 0x536DBF1E, 0x21E1B887, 
	0xDED0F0DC, 0xB4B1032D, 0x1D5AAF4A, 0xC56E12C3, 
	0x8C578DE8, 0xCBA564C9, 0xA67EEC36, 0x0837D2BF, 
	0x3D98D5B4, 0x1B06F225, 0xFF7EE1E2, 0x3640747B, 
	0x5E301440, 0x53A08741, 0x436FBC4E, 0xC9C333F7, 
	0x2727558C, 0x7F5CC01D, 0xFC83677A, 0xAFF10D33, 
	0x24836598, 0x3161F8B9, 0xDD748F66, 0x5B6CBC2F, 
	0xAD8FD064, 0x89EE4D15, 0xBBB2A012, 0xA086BCEB, 
	0x1BEAE1F0, 0x69F39931, 0x764DC57E, 0x17394B67, 
	0x4D51A63C, 0xF273790D, 0x35A2EBAA, 0x7EE463A3, 
	0xBC2BE948, 0x2B9B48A9, 0x2FC7BE96, 0x5FC9C19F, 
	0x3AD83714, 0x6FA02405, 0xDDB6AA42, 0xE648E15B, 
	0x1DB7DBA0, 0xF55AE721, 0x4D3ADAAE, 0xB3DAFED7, 
	0x5FFAE2EC, 0x96A42DFD, 0xFB9C3BDA, 0x21CF1613, 
	0x0F2C18F8, 0xAE705499, 0x650B79C6, 0x31C5E30F, 
	0x097D09C4, 0xAAAB76F5, 0x34CE0072, 0x27EDE1CB, 
	0xDAD20150, 0xADD57111, 0xC229FBDE, 0x8AFF4E47, 
	0x448E0B9C, 0x5C5DDEED, 0x4612580A, 0x05F82483, 
	0xBC1EF4A8, 0xB1C01C89, 0xF592C0F6, 0x6798207F, 
	0xEC494874, 0x795F45E5, 0xECFBA2A2, 0xBB9CBE3B, 
	0xF567104f, 0x47289407, 0x25683fa6, 0x2fde5836, 
};






class MaterialManager
{
	public:
	struct HashMap
	{
		unsigned m_id0;
		unsigned m_id1;
		unsigned m_key;
		int m_InterationEntry;
	};

	MaterialManager::MaterialManager(NewtonWorld* world, SoundManager* soudnManager)
	{
		int defaultMaterialId;

		// save sound manager
		m_soundManager = soudnManager;

		// Save the current User Data assigned to the world
		m_userData = NewtonWorldGetUserData(world);

		// set the new user Data;
		NewtonWorldSetUserData (world, this);

		// Save the Current Destructor Call back
		m_destrutorCallBack = NewtonWorldGetDestructorCallBack(world);

		// Set The Material Call Back 
		NewtonWorldSetDestructorCallBack (world, MaterialDestructor);


		// this Manager will use Default Material to handle all material integrations
		defaultMaterialId = NewtonMaterialGetDefaultGroupID (world);
		NewtonMaterialSetCollisionCallback (world, defaultMaterialId, defaultMaterialId, NULL, NULL, GenericContactProcess);

		// clear the default to material,
		memset (&m_defaultInteration, 0, sizeof (PhysicsMaterialInteration));

		// crate space for the the Database
		m_interactionCount = 0;
		m_maxInteractionSize = 8 * 2;
		m_InteractionTable = (PhysicsMaterialInteration*) malloc (m_maxInteractionSize * sizeof (PhysicsMaterialInteration));

		// create the HaspMap table;
		m_hapMapSize = 17;
		m_hapMap = (HashMap*) malloc (m_hapMapSize * sizeof (HashMap));
		memset (m_hapMap, 0, m_hapMapSize * sizeof (HashMap));
	}

	MaterialManager::~MaterialManager(void)
	{
		// destroy all used Memory;
		free (m_hapMap);
		free (m_InteractionTable);
	}

	static void MaterialDestructor (const NewtonWorld* newtonWorld)
	{
		MaterialManager* manager;

		// get the pointer to the Material Class
		manager = (MaterialManager*) NewtonWorldGetUserData(newtonWorld);

		// Installed the other Object so that other object can be destroyed too
		NewtonWorldSetUserData (newtonWorld, manager->m_userData);
		if (manager->m_destrutorCallBack) {
			manager->m_destrutorCallBack (newtonWorld);
		}

		// Destroy the Material Class
		delete manager;
	}

	unsigned GetHashCode (int id0, int id1, int hapMapSize) const
	{
		unsigned crc;
		unsigned val;

		if (id0 < id1) {
			int id;
			id = id0;
			id0 = id1;
			id1 = id;
		}

		crc = 0;
		val = randBits[((crc >> 24) ^ id0) & 0xff];
		crc = (crc << 8) ^ val;

		val = randBits[((crc >> 24) ^ id1) & 0xff];
		crc = (crc << 8) ^ val;

		val = randBits[((crc >> 24) ^ hapMapSize) & 0xff];
		crc = (crc << 8) ^ val;
		
		return crc;
	}

	int ReHachTheTable (int size)
	{
		HashMap *newMap;

		// allocate a new Has space
		newMap = (HashMap*) malloc (size * sizeof (HashMap));
		memset (newMap, 0, size * sizeof (HashMap));

		// copy each entry to the new hash map
		for (int i = 0; i < m_hapMapSize; i ++) {
			if (m_hapMap[i].m_key) {
				unsigned index;
				unsigned hashCode;
				hashCode = GetHashCode (m_hapMap[i].m_id0, m_hapMap[i].m_id1, size);
				index = hashCode % size;
				if (newMap[index].m_key) {
					// if there is a collision return with an error
					free (newMap);
					return 0;
				}
				newMap[index] = m_hapMap[i];
				newMap[index].m_key = hashCode;
			}
		}
		
		// delete old Hash map and set the pointer to the new map
		free (m_hapMap);
		m_hapMap = newMap;
		m_hapMapSize = size;

		return 1;
	}

	void AddInteraction (int materialId_0, int materialId_1, PhysicsMaterialInteration& interation)
	{
//		unsigned index;
		unsigned hashCode;

		//add the Material interaction to the end of the list
		if (m_interactionCount >= m_maxInteractionSize) {
			// the interaction pool is full we need to reallocate a larger Pool 
			PhysicsMaterialInteration* newPool;

			newPool = (PhysicsMaterialInteration*) malloc (2 * m_maxInteractionSize * sizeof (PhysicsMaterialInteration));
			memcpy (newPool, m_InteractionTable, m_interactionCount* sizeof (PhysicsMaterialInteration));
			free (m_InteractionTable);

			m_InteractionTable = newPool;
			m_maxInteractionSize *= 2;
		}

		m_InteractionTable[m_interactionCount] = interation;

		// find the entry into the HahMap table;
		hashCode = GetHashCode (materialId_0, materialId_1, m_hapMapSize);
//		index = hashCode % m_hapMapSize;

		for (int i = 0; i < MAX_HASH_COLLISION; i ++) {
			int entry = (hashCode + i) % m_hapMapSize;
			if (m_hapMap[entry].m_key == hashCode)  { 
				return;
			}
		}

		for (;;) {	
			for (int i = 0; i < MAX_HASH_COLLISION; i ++) {

				int entry = (hashCode + i) % m_hapMapSize;
				if (!m_hapMap[entry].m_key) {
					// save the Key information into the Hash Map
					m_hapMap[entry].m_key = hashCode;
					m_hapMap[entry].m_id0 = materialId_0;
					m_hapMap[entry].m_id1 = materialId_1;
					m_hapMap[entry].m_InterationEntry = m_interactionCount;
					m_interactionCount ++;
					return;
				}
			}
			for (int size = 2 * m_hapMapSize; !ReHachTheTable (size); size *= 2);	
			hashCode = GetHashCode (materialId_0, materialId_1, m_hapMapSize);
		}
	}

	
	const PhysicsMaterialInteration* GetMaterial (int id0, int id1) const
	{
		unsigned code;

		code = GetHashCode (id0, id1, m_hapMapSize);
		for (int i = 0; i < MAX_HASH_COLLISION; i ++) {
			unsigned index;
			index = (code + i) % m_hapMapSize;
			if (m_hapMap[index].m_key == code) {
				return &m_InteractionTable[m_hapMap[index].m_InterationEntry];
			}
		}

		return &m_defaultInteration;
	}
				

	static void GenericContactProcess (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
	{
		MaterialManager* manager;
		manager = (MaterialManager*) NewtonWorldGetUserData (NewtonBodyGetWorld (NewtonJointGetBody0(contactJoint)));
		manager->ContactProcess (contactJoint, timestep, threadIndex);
	}


	void ContactProcess (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
	{
		dFloat contactBestSpeed;
		dVector contactPosit;
		void* bestSound;
		NewtonBody* body0;
		NewtonBody* body1;

		bestSound = NULL;
		contactBestSpeed = 0.5f;
		body0 = NewtonJointGetBody0(contactJoint);
		body1 = NewtonJointGetBody1(contactJoint);
		for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = NewtonContactJointGetNextContact (contactJoint, contact)) {
			int id0;
			int id1;
			dFloat contactNormalSpeed;
			NewtonMaterial* material;
			const PhysicsMaterialInteration* appMaterial;

			// get the material for this contact;
			material = NewtonContactGetMaterial (contact);

			id0 = NewtonMaterialGetBodyCollisionID (material, body0);
			if (id0 == 0) {
				id0	= NewtonMaterialGetContactFaceAttribute (material);
			}
			id1 = NewtonMaterialGetBodyCollisionID (material, body1);
			if (id1 == 0) {
				id1	= NewtonMaterialGetContactFaceAttribute (material);
			}

			// get the material interaction for these two Ids from the hash map 
			appMaterial = GetMaterial (id0, id1);

			// set the physics material properties.
			NewtonMaterialSetContactElasticity (material, appMaterial->m_restitution);
			NewtonMaterialSetContactFrictionCoef (material, appMaterial->m_staticFriction, appMaterial->m_kineticFriction, 0);
			NewtonMaterialSetContactFrictionCoef (material, appMaterial->m_staticFriction, appMaterial->m_kineticFriction, 1);

			// do any other action the application desires.
			// for these toturial I am playing a sound effect.
			contactNormalSpeed = NewtonMaterialGetContactNormalSpeed (material);
			if (contactNormalSpeed > contactBestSpeed){
				// this will collect the strongest contact impact
				contactBestSpeed = contactNormalSpeed;
				dVector normal;
				contactBestSpeed = contactNormalSpeed;
				NewtonMaterialGetContactPositionAndNormal (material, body0, &contactPosit[0], &normal[0]);
				bestSound = appMaterial->m_impactSound;
			}
		}


		
		// if the strongest impact contact had a sound effect now play that effect.
		if (bestSound) {
			dFloat volume;
			dFloat dist2;

			// control sound volume based on camera distance to the contact
			dVector eyePoint (GetCameraEyePoint() - contactPosit);
			dist2 = eyePoint % eyePoint;
			if (dist2 < (MAX_SOUND_DISTANCE * MAX_SOUND_DISTANCE)) {
				volume = 1.0f;
				if (dist2 > (MIN_SOUND_DISTANCE * MIN_SOUND_DISTANCE)) {
					volume = 1.0f - (dSqrt (dist2) - MIN_SOUND_DISTANCE) / (MAX_SOUND_DISTANCE -  MIN_SOUND_DISTANCE);
				}
				// play this sound effect
				m_soundManager->Play (bestSound, volume, 0);
			}
		}
	}

	void* m_userData;						// user data in the work for destructor Chaining
	SoundManager* m_soundManager;
	NewtonDestroyWorld m_destrutorCallBack;

	int m_hapMapSize;
	int m_interactionCount;
	int m_maxInteractionSize;
	HashMap* m_hapMap;
	PhysicsMaterialInteration* m_InteractionTable; 
	PhysicsMaterialInteration m_defaultInteration;
};


// Create an advanced Material Manager class
void* CreateMaterialManager (NewtonWorld* world, SoundManager* soudnManager)
{
	return new MaterialManager (world, soudnManager);
}


// Set Default Material
void AddDefaultMaterial (void* manager, PhysicsMaterialInteration* interation)
{
	MaterialManager* manageClass = (MaterialManager*)manager;

	manageClass->m_defaultInteration = *interation;
}

// This Add and Interaction between two Material to the database 
void AddMaterilInteraction (void* manager, int materialId_0, int materialId_1, PhysicsMaterialInteration* interation)
{
	MaterialManager* manageClass = (MaterialManager*)manager;
	manageClass->AddInteraction (materialId_0, materialId_1, *interation);
}

