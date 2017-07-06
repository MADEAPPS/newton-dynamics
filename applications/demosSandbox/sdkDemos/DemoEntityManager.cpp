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

#include "toolbox_stdafx.h"
#include "NewtonDemos.h"

#include "SkyBox.h"
#include "DemoMesh.h"
#include "DemoEntity.h"
#include "DemoCamera.h"
#include "OpenGlUtil.h"
#include "PhysicsUtils.h"
#include "DebugDisplay.h"
#include "TargaToOpenGl.h"
#include "DemoEntityManager.h"
#include "DemoEntityListener.h"
#include "DemoCameraListener.h"
//#include "DemoVisualDebugerListener.h"
#include "dCustomPlayerControllerManager.h"
#include "dCustomVehicleControllerManager.h"


#ifdef _MACOSX_VER
	#include "CocoaOpenglGlue.h"
#endif

#define MAX_PHYSICS_FPS				120.0f


#define PROJECTILE_INITIAL_SPEED	20.0f

BEGIN_EVENT_TABLE (DemoEntityManager, wxGLCanvas)
	EVT_KEY_UP(DemoEntityManager::OnKeyUp)	
	EVT_KEY_DOWN(DemoEntityManager::OnKeyDown)
	EVT_MOUSE_EVENTS (DemoEntityManager::OnMouse)

	EVT_SIZE(DemoEntityManager::OnSize)
	EVT_PAINT(DemoEntityManager::OnPaint)
	EVT_IDLE(DemoEntityManager::OnIdle)
	EVT_ERASE_BACKGROUND(DemoEntityManager::OnEraseBackground)
END_EVENT_TABLE()

int DemoEntityManager::m_attributes[] = {WX_GL_DOUBLEBUFFER, WX_GL_RGBA, WX_GL_DEPTH_SIZE, 24, 0};


DemoEntityManager::ButtonKey::ButtonKey (bool state)
	:m_state(state)
	,m_memory0(false)
	,m_memory1(false)
{
}

bool DemoEntityManager::ButtonKey::UpdateTriggerButton (const NewtonDemos* const mainWin, int keyCode)
{
	m_memory0 = m_memory1;
	m_memory1 = mainWin->GetKeyState (keyCode);
	return !m_memory0 & m_memory1;
}

bool DemoEntityManager::ButtonKey::UpdateTriggerJoystick (const NewtonDemos* const mainWin, int buttonMask)
{
	m_memory0 = m_memory1;
	m_memory1 = buttonMask ? true : false;
	return !m_memory0 & m_memory1;
}

bool DemoEntityManager::ButtonKey::UpdatePushButton (const NewtonDemos* const mainWin, int keyCode)
{
	if (UpdateTriggerButton (mainWin, keyCode)) {
		m_state = ! m_state;
	}
	return m_state;
}


DemoEntityManager::DemoEntityManager(NewtonDemos* const parent)
	:wxGLCanvas(parent, wxID_ANY, m_attributes, wxDefaultPosition, wxSize(300, 300), wxSUNKEN_BORDER|wxFULL_REPAINT_ON_RESIZE, _("GLRenderCanvas"))
	,dList <DemoEntity*>() 
	,m_mainWindow(parent)
	,m_world(NULL)
	,m_sky(NULL)
	,m_microsecunds(0)
	,m_currentListenerTimestep(0.0f)
	,m_physicsUpdate(true) 
	,m_reEntrantUpdate (false)
	,m_renderHoodContext(NULL)
	,m_renderHood(NULL)
	,m_font(0)
	,m_fontImage(0)
	,m_cameraManager(NULL)
    ,m_tranparentHeap()
//	,m_visualDebugger(NULL)
	,m_mainThreadGraphicsTime(0.0f)
	,m_mainThreadPhysicsTime(0.0f)
{
	// initialized the physics world for the new scene
	Cleanup ();

	ResetTimer();

	dTimeTrackerSetThreadName ("mainThread");
	m_context = new wxGLContext(this);

/*
	dFloat A[2][2];
	dFloat x[2];
	dFloat b[2];
	dFloat l[2];
	dFloat h[2];

	A[0][0] = 2.0f;
	A[0][1] = 1.0f;
	A[1][0] = 1.0f;
	A[1][1] = 2.0f;
	b[0] = 1.0f;
	b[1] = 1.0f;
	x[0] = 1;
	x[1] = 2;
	
	l[0] = 0.0f;
	l[1] = 0.0f;
	h[0] = 0.25f;
	h[1] = 1.0f;
	
	dMatrixTimeVector(2, &A[0][0], x, b);
	dSolveDantzigLCP(2, &A[0][0], x, b, l, h);
*/

/*
for (int ii = 0; ii < 10000; ii++) {
	NewtonDestroy(NewtonCreate());
}
*/

}


DemoEntityManager::~DemoEntityManager(void)
{
	// is we are run asynchronous we need make sure no update in on flight.
	if (m_world) {
		NewtonWaitForUpdateToFinish (m_world);
	}

	glDeleteLists(m_font, 96);	
	ReleaseTexture(m_fontImage);

	Cleanup ();

	// destroy the empty world
	if (m_world) {
		NewtonDestroy (m_world);
		m_world = NULL;
	}
	dAssert (NewtonGetMemoryUsed () == 0);

	delete m_context;
}


void DemoEntityManager::Cleanup ()
{
	// is we are run asynchronous we need make sure no update in on flight.
	if (m_world) {
		NewtonWaitForUpdateToFinish (m_world);
	}

	// destroy all remaining visual objects
	while (dList<DemoEntity*>::GetFirst()) {
		RemoveEntity (dList<DemoEntity*>::GetFirst());
	}

	m_sky = NULL;

	// destroy the Newton world
	if (m_world) {
		// get serialization call back before destroying the world
		NewtonDestroy (m_world);
		m_world = NULL;
	}

	//	memset (&demo, 0, sizeof (demo));
	// check that there are no memory leak on exit
	dAssert (NewtonGetMemoryUsed () == 0);

	// create the newton world
	m_world = NewtonCreate();

	// link the work with this user data
	NewtonWorldSetUserData(m_world, this);

	// set joint serialization call back
	dCustomJoint::Initalize(m_world);

	// add all physics pre and post listeners
	//	m_preListenerManager.Append(new DemoVisualDebugerListener("visualDebuger", m_world));
	new DemoEntityListener (this);
	m_cameraManager = new DemoCameraListener(this);
	//	m_postListenerManager.Append (new DemoAIListener("aiManager"));

	// set the default parameters for the newton world
	// set the simplified solver mode (faster but less accurate)
	NewtonSetSolverModel (m_world, 4);

	// newton 300 does not have world size, this is better controlled by the client application
	//dVector minSize (-500.0f, -500.0f, -500.0f);
	//dVector maxSize ( 500.0f,  500.0f,  500.0f);
	//NewtonSetWorldSize (m_world, &minSize[0], &maxSize[0]); 

	// set the performance track function
	//NewtonSetPerformanceClock (m_world, dRuntimeProfiler::GetTimeInMicrosenconds);

	// clean up all caches the engine have saved
	NewtonInvalidateCache (m_world);

	// Set the Newton world user data
	NewtonWorldSetUserData(m_world, this);


	// we start without 2d render
	m_renderHood = NULL;
	m_renderHoodContext = NULL;
}

void DemoEntityManager::ResetTimer()
{
	dResetTimer();
	m_microsecunds = dGetTimeInMicrosenconds ();
}



void DemoEntityManager::RemoveEntity (dListNode* const entNode)
{
	DemoEntity* const entity = entNode->GetInfo();
	entity->Release();
	Remove(entNode);
}

void DemoEntityManager::RemoveEntity (DemoEntity* const ent)
{
	for (dListNode* node = dList<DemoEntity*>::GetFirst(); node; node = node->GetNext()) {
		if (node->GetInfo() == ent) {
			RemoveEntity (node);
			break;
		}
	}
}

void DemoEntityManager::PushTransparentMesh (const DemoMeshInterface* const mesh)
{
    dMatrix matrix;
    glGetFloat (GL_MODELVIEW_MATRIX, &matrix[0][0]);
    TransparentMesh entry (matrix, (DemoMesh*) mesh);
    m_tranparentHeap.Push (entry, matrix.m_posit.m_z);
}

void DemoEntityManager::CreateOpenGlFont()
{
	FT_Library  library;
	FT_Error error = FT_Init_FreeType (&library);
	if ( !error )
	{
		char fileName[2048];
		//GetWorkingFileName ("arial.ttf", fileName);
		//GetWorkingFileName ("calibri.ttf", fileName);
		dGetWorkingFileName ("courbd.ttf", fileName);
				

		FT_Face face[96];   
		int withInPixels = 12;
		int heightInPixels = 16;

		int width = 0;
		int height = 0;
		for (int ch = 0; ch < 96; ch ++) {
			// Load The Glyph For Our Character.
			error = FT_New_Face( library, fileName, 0, &face[ch] );
			dAssert (!error);

			FT_Face bitmap = face[ch];   

			error = FT_Set_Char_Size(bitmap, withInPixels * 64, heightInPixels * 64, 96, 96);
			dAssert (!error);

			FT_UInt index = FT_Get_Char_Index( face[ch], ch + ' ');
			//FT_UInt index = FT_Get_Char_Index (bitmap, 'A');

			error = FT_Load_Glyph (bitmap, index, FT_LOAD_DEFAULT );
			dAssert (!error);

			error = FT_Render_Glyph (bitmap->glyph, FT_RENDER_MODE_NORMAL); 
			dAssert (!error);

			const FT_Glyph_Metrics& metrics = bitmap->glyph->metrics;
			int w = metrics.width / 64;
			int h = metrics.height / 64;

			width += w;
			height = (height > h) ? height : h;
		}

		int imageWidth = dTwosPower (width);
		int imageHeight = dTwosPower (height);

		char* const image = new char[2 * imageWidth * imageHeight];
		memset (image, 0, 2 * imageWidth * imageHeight);

		int maxWidth = 0;
		int imageBase = 0;
		
		for (int ch = 0; ch < 96; ch ++) {
			FT_Face bitmap = face[ch];   
			FT_GlyphSlot slot = bitmap->glyph;

			const FT_Glyph_Metrics& metrics = slot->metrics;
			int w = metrics.width / 64;
			int h = metrics.height / 64;

			maxWidth = (w > maxWidth) ? w : maxWidth;
			if (w) {
				const unsigned char* const buffer = slot->bitmap.buffer;
				int pitch =  slot->bitmap.pitch;

				int posit = imageBase;
				for (int j = 0; j < h; j ++) {
					for (int i = 0; i < w; i ++) {
						int color = buffer[j * pitch + i];
						image[posit + i * 2 + 0] = color;
						image[posit + i * 2 + 1] = color;
					}
					posit += imageWidth * 2;
				}
				imageBase += w * 2;
			}
		}

		// make th open gl display list here
	    m_fontImage = LoadImage("fontTexture", image, imageWidth, imageHeight, m_luminace);

		m_font = glGenLists(96);
		glBindTexture(GL_TEXTURE_2D, m_fontImage);

		imageBase = 0;
		for (int ch = 0; ch < 96; ch ++) {
			FT_Face bitmap = face[ch];   
			FT_GlyphSlot slot = bitmap->glyph;
			const FT_Glyph_Metrics& metrics = slot->metrics;

			glNewList(m_font + ch, GL_COMPILE);
			glPushMatrix();
//			glTranslatef(slot->bitmap_left, 64 - slot->bitmap_top, 0);
			glTranslatef(slot->bitmap_left, - slot->bitmap_top, 0);

			dFloat w = dFloat (metrics.width / 64);
			dFloat h = dFloat (metrics.height / 64);

			if (w) {
				dFloat u0 = dFloat (imageBase) / imageWidth;
				dFloat u1 = dFloat (imageBase + w - 1.0f) / imageWidth;

				dFloat v0 = 0.0f;
				dFloat v1 = (h - 1.0f) / imageHeight;

				glBegin(GL_QUADS);

				glTexCoord2d (u0, v0); 
				glVertex2i(0, 0);

				glTexCoord2d (u0, v1); 
				glVertex2i(0, h - 1);

				glTexCoord2d (u1, v1); 
				glVertex2i (w - 1, h - 1);

				glTexCoord2d (u1, v0); 
				glVertex2i (w - 1, 0);
				glEnd();

				imageBase += w;
			}
			glPopMatrix();
			
			//glTranslatef(maxWidth, 0, 0);
			glTranslatef(metrics.horiAdvance / 64, 0, 0);

			glEndList();
			FT_Done_Face(bitmap);
		}

		delete[] image; 

		// destroy the free type library	
		FT_Done_FreeType (library);
	}
}


void DemoEntityManager::InitGraphicsSystem()
{
	wxGLCanvas::SetCurrent(*m_context);
	
	GLenum err = glewInit();
	
	// if Glew doesn't initialize correctly.
	if (err != GLEW_OK) {
	  //wxMessageBox(wxString(_("GLEW Error: ")) + wxString(_((char*)glewGetErrorString(err))), _("ERROR"), wxOK | wxICON_EXCLAMATION);
	}
	
#if defined (_MSC_VER)
	if (wglSwapIntervalEXT) {
		wglSwapIntervalEXT(0);
	}
#elif (defined (_POSIX_VER) || defined (_POSIX_VER_64))
	if (glXSwapIntervalSGI) {
		glXSwapIntervalSGI(0);  //NOTE check for GLX_SGI_swap_control extension : http://www.opengl.org/wiki/Swap_Interval#In_Linux_.2F_GLXw
	}
#elif defined(_MACOSX_VER)
    //wglSwapIntervalEXT (GetContext()->GetWXGLContext());
	wglSwapIntervalEXT (m_context->GetWXGLContext());
#endif

	// initialize free type library
	CreateOpenGlFont();
}


void DemoEntityManager::SerializeFile (void* const serializeHandle, const void* const buffer, int size)
{
	// check that each chunk is a multiple of 4 bytes, this is useful for easy little to big Indian conversion
	dAssert ((size & 0x03) == 0);
	fwrite (buffer, size, 1, (FILE*) serializeHandle);
}

void DemoEntityManager::DeserializeFile (void* const serializeHandle, void* const buffer, int size)
{
	// check that each chunk is a multiple of 4 bytes, this is useful for easy little to big Indian conversion
	dAssert ((size & 0x03) == 0);
	fread (buffer, size, 1, (FILE*) serializeHandle);
}


void DemoEntityManager::BodySerialization (NewtonBody* const body, void* const bodyUserData, NewtonSerializeCallback serializeCallback, void* const serializeHandle)
{
	// here the use can save information of this body, ex:
	// a string naming the body,  
	// serialize the visual mesh, or save a link to the visual mesh
	// save labels for looking up the body call backs

	// for the demos I will simple write three stream to identify what body it is, the application can do anything
	const char* const bodyIndentification = "gravityBody\0\0\0\0";
	int size = (strlen (bodyIndentification) + 3) & -4;
	serializeCallback (serializeHandle, &size, sizeof (size));
	serializeCallback (serializeHandle, bodyIndentification, size);
}


void DemoEntityManager::BodyDeserialization (NewtonBody* const body, void* const bodyUserData, NewtonDeserializeCallback deserializecallback, void* const serializeHandle)
{
	int size;
	char bodyIndentification[256];
	
	deserializecallback (serializeHandle, &size, sizeof (size));
	deserializecallback (serializeHandle, bodyIndentification, size);

	// get the world and the scene form the world user data
	NewtonWorld* const world = NewtonBodyGetWorld(body);
	DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

	// here we attach a visual object to the entity, 
	dMatrix matrix;
	NewtonBodyGetMatrix(body, &matrix[0][0]);
	DemoEntity* const entity = new DemoEntity(matrix, NULL);
	scene->Append (entity);

	NewtonBodySetUserData (body, entity);
	NewtonBodySetTransformCallback(body, DemoEntity::TransformCallback);
	NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);
	NewtonCollision* const collision = NewtonBodyGetCollision(body);

	#ifdef USE_STATIC_MESHES_DEBUG_COLLISION
		if (NewtonCollisionGetType(collision) == SERIALIZE_ID_TREE) {
			NewtonStaticCollisionSetDebugCallback (collision, ShowMeshCollidingFaces);
		}
	#endif

	//for visual mesh we will collision mesh and convert it to a visual mesh using NewtonMesh 
	dTree <DemoMeshInterface*, const void*>* const cache = (dTree <DemoMeshInterface*, const void*>*)bodyUserData;
	dTree <DemoMeshInterface*, const void*>::dTreeNode* node = cache->Find(NewtonCollisionDataPointer (collision));
	if (!node) {
		DemoMeshInterface* mesh = new DemoMesh(bodyIndentification, collision, NULL, NULL, NULL);
		node = cache->Insert(mesh, NewtonCollisionDataPointer (collision));
	} else {
		node->GetInfo()->AddRef();
	}
	
	DemoMeshInterface* const mesh = node->GetInfo();
	entity->SetMesh(mesh, dGetIdentityMatrix());
	mesh->Release();
}

void DemoEntityManager::SerializedPhysicScene (const char* const name)
{
	NewtonSerializeToFile (m_world, name, BodySerialization, NULL);
}

void DemoEntityManager::DeserializedPhysicScene (const char* const name)
{
	// add the sky
	CreateSkyBox();

	dQuaternion rot;
	dVector origin (-30.0f, 10.0f, 10.0f, 0.0f);
	SetCameraMatrix(rot, origin);

	dTree <DemoMeshInterface*, const void*> cache;
	NewtonDeserializeFromFile (m_world, name, BodyDeserialization, &cache);
}


void DemoEntityManager::CreateSkyBox()
{
	if (!m_sky) {
		m_sky = new SkyBox();
		Append(m_sky);
	}
}

DemoCamera* DemoEntityManager::GetCamera() const
{
	return m_cameraManager->GetCamera();
}

void DemoEntityManager::SetCameraMouseLock (bool state)
{
	m_cameraManager->SetCameraMouseLock(state);
}

void DemoEntityManager::SetCameraMatrix (const dQuaternion& rotation, const dVector& position)
{
	m_cameraManager->SetCameraMatrix(this, rotation, position);
}

void DemoEntityManager::Set2DDisplayRenderFunction (RenderHoodCallback callback, void* const context)
{
	m_renderHood = callback;
	m_renderHoodContext = context;
}



void DemoEntityManager::LoadVisualScene(dScene* const scene, EntityDictionary& dictionary)
{
	// load all meshes into a Mesh cache for reuse
	dTree<DemoMeshInterface*, dScene::dTreeNode*> meshDictionary;
	for (dScene::dTreeNode* node = scene->GetFirstNode (); node; node = scene->GetNextNode (node)) {
		dNodeInfo* info = scene->GetInfoFromNode(node);
		if (info->GetTypeId() == dMeshNodeInfo::GetRttiType()) {
			DemoMeshInterface* const mesh = new DemoMesh(scene, node);
			meshDictionary.Insert(mesh, node);
		}
	}

	// create an entity for every root node in the mesh
	// a root node or scene entity is a dSceneNodeInfo with a direct link to the root of the dScene node.
	dScene::dTreeNode* const root = scene->GetRootNode();
	for (void* child = scene->GetFirstChildLink(root); child; child = scene->GetNextChildLink (root, child)) {
		dScene::dTreeNode* node = scene->GetNodeFromLink(child);
		dNodeInfo* info = scene->GetInfoFromNode(node);
		if (info->GetTypeId() == dSceneNodeInfo::GetRttiType()) {
			// we found a root dSceneNodeInfo, convert it to a Scene entity and load all it children 
			DemoEntity* const entityRoot = new DemoEntity (*this, scene, node, meshDictionary, dictionary);
			Append(entityRoot);
		}
	}

	// release all meshes before exiting
	dTree<DemoMeshInterface*, dScene::dTreeNode*>::Iterator iter (meshDictionary);
	for (iter.Begin(); iter; iter++) {
		DemoMeshInterface* const mesh = iter.GetNode()->GetInfo();
		mesh->Release();
	}
}


void DemoEntityManager::LoadScene (const char* const fileName)
{
	dScene database (GetNewton());

	database.Deserialize(fileName);

	// this will apply all global the scale to the mesh
	database.FreezeScale();
	// this will apply all local scale and transform to the mesh
	//database.FreezePivot();

	// Load the Visual Scene
	EntityDictionary entDictionary;
	LoadVisualScene(&database, entDictionary);

	//Load the physics world
	dList<NewtonBody*> bodyList;
	database.SceneToNewtonWorld(m_world, bodyList);

	// bind every rigidBody loaded to the scene entity
	for (dList<NewtonBody*>::dListNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext()) {
		// find the user data and set to the visual entity in the scene
		NewtonBody* const body = bodyNode->GetInfo();
		dScene::dTreeNode* const sceneNode = (dScene::dTreeNode*)NewtonBodyGetUserData(body);
		DemoEntity* const entity = entDictionary.Find(sceneNode)->GetInfo();
		NewtonBodySetUserData(body, entity);

		// see if this body have some special setups
		dScene::dTreeNode* const node = database.FindChildByType(sceneNode, dRigidbodyNodeInfo::GetRttiType());
		dAssert (node);
		dRigidbodyNodeInfo* const bodyData = (dRigidbodyNodeInfo*) database.GetInfoFromNode(node);
		dVariable* bodyType = bodyData->FindVariable("rigidBodyType");

		// set the default call backs
		if (!bodyType || !strcmp (bodyType->GetString(), "default gravity")) {
			NewtonBodySetTransformCallback(body, DemoEntity::TransformCallback);
			NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);
			NewtonBodySetDestructorCallback (body, PhysicsBodyDestructor);
		}
	}

	// clean up all caches the engine have saved
	NewtonInvalidateCache (m_world);
}


dFloat DemoEntityManager::GetPhysicsTime()
{
	return m_mainThreadPhysicsTime;
}



int DemoEntityManager::Print (const dVector& color, dFloat x, dFloat y, const char *fmt, ... ) const
{
	glColor3f(color.m_x, color.m_y, color.m_z);

	glPushMatrix();

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0, GetWidth(), GetHeight(), 0.0, 0.0, 1.0);
 
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glTranslated(x, y, 0);
//	glRasterPos2f(x, y + 16);

	va_list argptr;
	char string[1024];

	va_start (argptr, fmt);
	vsprintf (string, fmt, argptr);
	va_end( argptr );

    glDisable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glBlendFunc (GL_SRC_COLOR, GL_ONE_MINUS_SRC_COLOR);      
	
	glBindTexture(GL_TEXTURE_2D, m_fontImage);

	glPushAttrib(GL_LIST_BIT);
	glListBase(m_font - 32);	
	int lenght = (int) strlen (string);
	glCallLists (lenght, GL_UNSIGNED_BYTE, string);	
	glPopAttrib();				

	glDisable(GL_BLEND);

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glLoadIdentity();

	return y;
}


void DemoEntityManager::UpdatePhysics(dFloat timestep)
{
	// update the physics
	if (m_world) {

		dFloat timestepInSecunds = 1.0f / MAX_PHYSICS_FPS;
		unsigned64 timestepMicrosecunds = unsigned64 (timestepInSecunds * 1000000.0f);

		unsigned64 currentTime = dGetTimeInMicrosenconds ();
		unsigned64 nextTime = currentTime - m_microsecunds;
		if (nextTime > timestepMicrosecunds * 2) {
			m_microsecunds = currentTime - timestepMicrosecunds * 2;
			nextTime = currentTime - m_microsecunds;
		}

		//while (nextTime >= timestepMicrosecunds) 
		if (nextTime >= timestepMicrosecunds) 
		{
			unsigned64 time0 = dGetTimeInMicrosenconds ();

			dTimeTrackerEvent(__FUNCTION__);
			// run the newton update function
			if (!m_reEntrantUpdate) {
				m_reEntrantUpdate = true;
				if (m_physicsUpdate && m_world) {

					ClearDebugDisplay(m_world);

					// update the physics world
					if (!m_mainWindow->m_physicsUpdateMode) {
						NewtonUpdate (m_world, timestepInSecunds);
					} else {
						NewtonUpdateAsync(m_world, timestepInSecunds);
					}
				}
				m_reEntrantUpdate = false;
			}
			nextTime -= timestepMicrosecunds;
			m_microsecunds += timestepMicrosecunds;

			unsigned64 time1 = dGetTimeInMicrosenconds ();
			m_mainThreadPhysicsTime = dFloat ((time1 - time0) / 1000000.0f);
		}
	}
}

dFloat DemoEntityManager::CalculateInteplationParam () const
{
	unsigned64 timeStep = dGetTimeInMicrosenconds () - m_microsecunds;		
	dFloat param = (dFloat (timeStep) * MAX_PHYSICS_FPS) / 1.0e6f;
	dAssert (param >= 0.0f);
	if (param > 1.0f) {
		param = 1.0f;
	}
	return param;
}

void DemoEntityManager::OnKeyUp( wxKeyEvent &event )
{
	m_mainWindow->KeyUp(event);
}


void DemoEntityManager::OnKeyDown( wxKeyEvent &event )
{
	m_mainWindow->KeyDown(event);
}

void DemoEntityManager::OnMouse(wxMouseEvent &event)
{
	m_mainWindow->MouseAction(event);
}


void DemoEntityManager::OnSize(wxSizeEvent& event)
{
	// this is also necessary to update the context on some platforms
	wxGLCanvas::OnSize(event);
}

void DemoEntityManager::OnEraseBackground(wxEraseEvent& WXUNUSED(event))
{
	// Do nothing, to avoid flashing on MSW
}


void DemoEntityManager::OnIdle(wxIdleEvent& event)
{
	wxClientDC dc(this);
	RenderFrame ();
	dTimeTrackerUpdate();
	event.RequestMore(); // render continuously, not only once on idle
}

void DemoEntityManager::OnPaint (wxPaintEvent& WXUNUSED(event))
{
	wxPaintDC dc(this);
	RenderFrame ();
}

void DemoEntityManager::RenderFrame ()
{
	dTimeTrackerEvent(__FUNCTION__);

	// Make context current
	if (m_mainWindow->m_suspendVisualUpdates) {
		return;
	}

	dFloat timestep = dGetElapsedSeconds();	
	m_mainWindow->CalculateFPS(timestep);

	// update the the state of all bodies in the scene
	UpdatePhysics(timestep);

	// Get the interpolated location of each body in the scene
	m_cameraManager->InterpolateMatrices (this, CalculateInteplationParam());

	// Our shading model--Goraud (smooth). 
	glShadeModel (GL_SMOOTH);

	// Culling. 
	glCullFace (GL_BACK);
	glFrontFace (GL_CCW);
	glEnable (GL_CULL_FACE);

	//	glEnable(GL_DITHER);

	// z buffer test
	glEnable(GL_DEPTH_TEST);
	glDepthFunc (GL_LEQUAL);

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_FASTEST);

	glClearColor (0.5f, 0.5f, 0.5f, 0.0f );
	//glClear( GL_COLOR_BUFFER_BIT );
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	// set default lightning
	//	glDisable(GL_BLEND);
	glEnable (GL_LIGHTING);

	// make sure the model view matrix is set to identity before setting world space ligh sources
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	dFloat cubeColor[] = { 1.0f, 1.0f, 1.0f, 1.0 };
	glMaterialParam(GL_FRONT, GL_SPECULAR, cubeColor);
	glMaterialParam(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cubeColor);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

	// one light form the Camera eye point
	GLfloat lightDiffuse0[] = { 0.5f, 0.5f, 0.5f, 0.0 };
	GLfloat lightAmbient0[] = { 0.0f, 0.0f, 0.0f, 0.0 };
	dVector camPosition (m_cameraManager->GetCamera()->m_matrix.m_posit);
	GLfloat lightPosition0[] = { GLfloat(camPosition.m_x), GLfloat(camPosition.m_y), GLfloat(camPosition.m_z)};

	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition0);
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse0);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightDiffuse0);
	glEnable(GL_LIGHT0);


	// set just one directional light
	GLfloat lightDiffuse1[] = { 0.7f, 0.7f, 0.7f, 0.0 };
	GLfloat lightAmbient1[] = { 0.2f, 0.2f, 0.2f, 0.0 };
	GLfloat lightPosition1[] = { -500.0f, 200.0f, 500.0f, 0.0 };

	glLightfv(GL_LIGHT1, GL_POSITION, lightPosition1);
	glLightfv(GL_LIGHT1, GL_AMBIENT, lightAmbient1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, lightDiffuse1);
	glLightfv(GL_LIGHT1, GL_SPECULAR, lightDiffuse1);
	glEnable(GL_LIGHT1);

	// update Camera
	m_cameraManager->GetCamera()->SetViewMatrix(GetWidth(), GetHeight());

	// render all entities
	if (m_mainWindow->m_hideVisualMeshes) {
		if (m_sky) {
			glPushMatrix();	
			m_sky->Render(timestep, this);
			glPopMatrix();
		}

	} else {
		for (dListNode* node = dList<DemoEntity*>::GetFirst(); node; node = node->GetNext()) {
			DemoEntity* const entity = node->GetInfo();
			glPushMatrix();	
			entity->Render(timestep, this);
			glPopMatrix();
		}
	}

	if (m_tranparentHeap.GetCount()) {
		dMatrix modelView;
		glGetFloat (GL_MODELVIEW_MATRIX, &modelView[0][0]);
		while (m_tranparentHeap.GetCount()) {
			const TransparentMesh& transparentMesh = m_tranparentHeap[0];
			glLoadIdentity();
			glLoadMatrix(&transparentMesh.m_matrix[0][0]);
			transparentMesh.m_mesh->RenderTransparency();
			m_tranparentHeap.Pop();
		}
		glLoadMatrix(&modelView[0][0]);
	}


	m_cameraManager->RenderPickedTarget ();

	if (m_mainWindow->m_showContactPoints) {
		RenderContactPoints (GetNewton());
	}

	if (m_mainWindow->m_showNormalForces) {
		RenderNormalForces (GetNewton());
	}

	if (m_mainWindow->m_showNormalForces) {
//	if (1) {
		// see if there is a vehicle controller and 
		void* const vehListerNode = NewtonWorldGetPreListener(GetNewton(), VEHICLE_PLUGIN_NAME);
		if (vehListerNode) {
			dCustomVehicleControllerManager* const manager = (dCustomVehicleControllerManager*)NewtonWorldGetListenerUserData(GetNewton(), vehListerNode);
			manager->Debug();
		}

		void* const characterListerNode = NewtonWorldGetPreListener(GetNewton(), PLAYER_PLUGIN_NAME);
		if (characterListerNode) {
			dCustomPlayerControllerManager* const manager = (dCustomPlayerControllerManager*)NewtonWorldGetListenerUserData(GetNewton(), characterListerNode);
			manager->Debug();
		}
	}



	if (m_mainWindow->m_showAABB) {
		RenderAABB (GetNewton());
	}

	if (m_mainWindow->m_showCenterOfMass) {
		RenderCenterOfMass (GetNewton());
	}

	if (m_mainWindow->m_showJoints) {
		RenderJointsDebugInfo (GetNewton(), 0.5f);
	}


   DEBUG_DRAW_MODE mode = m_solid;
   if (m_mainWindow->m_debugDisplayMode) {
      mode = (m_mainWindow->m_debugDisplayMode == 1) ? m_solid : m_lines;
      DebugRenderWorldCollision (GetNewton(), mode);
   }

	if (m_mainWindow->m_showStatistics) {
		dVector color (1.0f, 1.0f, 1.0f, 0.0f);
		Print (color, 10,  20, "render fps: %7.2f", m_mainWindow->m_fps);
		Print (color, 10,  42, "physics time on main thread: %7.2f ms", GetPhysicsTime() * 1000.0f);
		Print (color, 10,  64, "total memory: %d kbytes", NewtonGetMemoryUsed() / (1024));
		Print (color, 10,  86, "number of bodies: %d", NewtonWorldGetBodyCount(GetNewton()));
		Print (color, 10, 108, "number of threads: %d", NewtonGetThreadsCount(GetNewton()));
		Print (color, 10, 130, "auto sleep: %s", m_mainWindow->m_autoSleepState ? "on" : "off");
	}

	//int lineNumber = 130 + 22;
	int lineNumber = 30;
	if (m_renderHood) {

		// set display for 2d render mode

		dFloat width = GetWidth();
		dFloat height = GetHeight();

		glColor3f(1.0, 1.0, 1.0);

		glPushMatrix();
		glMatrixMode(GL_PROJECTION);

		glLoadIdentity();
		gluOrtho2D(0, width, 0, height);
		
			glPushMatrix();
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();

			glDisable(GL_LIGHTING);
			glDisable(GL_DEPTH_TEST);
			glEnable(GL_TEXTURE_2D);	

			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

			// render 2d display
			m_renderHood (this, m_renderHoodContext, lineNumber);

			// restore display mode
			glMatrixMode(GL_PROJECTION);
			glPopMatrix();


		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	}


	// draw everything and swap the display buffer
	glFlush();

	// Swap
	SwapBuffers();
}
