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

#ifndef __SCENE_MANAGER__H
#define __SCENE_MANAGER__H

#define MAX_ENTITY_COUNT		 1024 	

class Entity;
class SoundManager;

class SceneManager
{
	public:
	SceneManager(void);
	virtual ~SceneManager(void);

	void CreateScene ();
	Entity* CreateEntity();

	void Render ();
	SoundManager* GetSoundManager() const;
	void SetIntepolationParam(dFloat param);
	

	protected:
	int m_entCount;
	dFloat m_intepolationParam;
	SoundManager* m_soundManager;
	Entity* m_entityArray[MAX_ENTITY_COUNT];

};

#endif
