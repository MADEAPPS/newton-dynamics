/* Copyright (c) <2003-2022> <Newton Game Dynamics>
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely
 */

package com.example.androidapp;

import android.opengl.GLES30;
import android.opengl.GLSurfaceView;
import android.content.res.AssetManager;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import com.javaNewton.nWorld;
import com.javaNewton.nMatrix;
import com.example.androidapp.Demos.BasicRigidBodies;

public class RenderScene implements GLSurfaceView.Renderer
{
    public float GetTimestep()
    {
        return m_timestep;
    }

    public nWorld GetWorld()
    {
        return m_world;
    }

    public SceneObject GetRoot()
    {
        return m_root;
    }

    public SceneCamera GetCamera()
    {
        return m_camera;
    }

    public void SetAssetManager(AssetManager assetManager)
    {
        m_assetManager = assetManager;
    }

    public AssetManager GetAssetManager()
    {
        return m_assetManager;
    }

    @Override
    public void onSurfaceCreated(GL10 unused, EGLConfig config)
    {
        // Set the background frame color
        GLES30.glCullFace (GLES30.GL_BACK);
        GLES30.glFrontFace (GLES30.GL_CCW);
        GLES30.glEnable (GLES30.GL_CULL_FACE);
        GLES30.glClearColor(0.5f, 0.5f, 0.5f, 1.0f);

        m_screenWidth = 0;
        m_screenHeight = 0;
        m_timestep = 1.0f / 60.0f;
        m_shaderCache = new ShaderCache(this);
        m_textureCache = new SceneMeshTextureCache(GetAssetManager());

        m_camera = new SceneCamera();
        m_root = new SceneObject();
        m_world = new nWorld();
        m_world.SetSubSteps(2);

        // load the first scene
        LoadScene();
        m_renderInitialized = true;
    }

    public void DestroyScene()
    {
        m_world.CleanUp();
        if (m_demo != null)
        {
            m_demo.CleanUp(this);
        }
        if (m_root != null)
        {
            m_root.CleanUp(this);
        }
        m_textureCache.Clear();

        m_root = null;
        m_demo = null;
        m_world = null;
        m_camera = null;
        m_textureCache = null;
   }

    public void AddSceneObject(SceneObject object)
    {
       object.AttachToParent(m_root);
    }

    @Override
    public void onDrawFrame(GL10 unused)
    {
        switch(m_renderState)
        {
            case m_renderSceneState:
            {
                RenderFrame();
                break;
            }

            case m_loadSceneState:
            {
                LoadScene();
                break;
            }

            case m_idleState:
            default:
            {
                GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT | GLES30.GL_DEPTH_BUFFER_BIT);
                break;
            }
        }
    }

    @Override
    public void onSurfaceChanged(GL10 unused, int width, int height)
    {
        m_screenWidth = width;
        m_screenHeight = height;
    }

    public ShaderCache GetShaderCache()
    {
        return m_shaderCache;
    }

    public SceneMeshTextureCache GetTextureCache()
    {
        return m_textureCache;
    }

    public static void checkGlError(String glOperation)
    {
        int error;
        while ((error = GLES30.glGetError()) != GLES30.GL_NO_ERROR)
        {
            //Log.e(TAG, glOperation + ": glError " + error);
            System.out.println(glOperation + ": glError " + error);
            throw new RuntimeException(glOperation + ": glError " + error);
        }
    }

    private void RenderFrame()
    {
        m_world.Sync();
        m_world.Update(m_timestep);

        GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT | GLES30.GL_DEPTH_BUFFER_BIT);

        m_camera.SetViewMatrix(m_screenWidth, m_screenHeight);

        GLES30.glCullFace (GLES30.GL_BACK);
        GLES30.glFrontFace (GLES30.GL_CCW);
        GLES30.glEnable (GLES30.GL_CULL_FACE);

        //	glEnable(GL_DITHER);
        // z buffer test
        GLES30.glEnable(GLES30.GL_DEPTH_TEST);
        GLES30.glDepthFunc (GLES30.GL_LEQUAL);

        //GLES30.glHint(GLES30.GL_PERSPECTIVE_CORRECTION_HINT, GLES30.GL_FASTEST);
        //GLES30.glHint(GLES30.GL_POLYGON_SMOOTH_HINT, GLES30.GL_FASTEST);
        // one light from the Camera eye point
        //nVector camPosition = m_camera.GetMatrix().GetPosition();
        //GLfloat lightDiffuse1[] = { 0.5f, 0.5f, 0.5f, 0.0f };
        //GLfloat lightAmbient1[] = { 0.0f, 0.0f, 0.0f, 0.0f };
        //GLfloat lightSpecular1[] = { 0.0f, 0.0f, 0.0f, 0.0f };
        //GLfloat lightPosition1[] = {0.0f, 0.0f, 0.0f, 1.0f};
        //glMaterialf(GL_FRONT, GL_SHININESS, 60.0f);
        //glLightfv(GL_LIGHT1, GL_POSITION, lightPosition1);
        //glLightfv(GL_LIGHT1, GL_AMBIENT, lightAmbient1);
        //glLightfv(GL_LIGHT1, GL_DIFFUSE, lightDiffuse1);
        //glLightfv(GL_LIGHT1, GL_SPECULAR, lightSpecular1);
        //glEnable(GL_LIGHT1);

        nMatrix matrix = new nMatrix();
        m_root.Render(this, matrix);
    }

    private void LoadScene()
    {
        if (m_demo != null)
        {
            m_demo.CleanUp(this);
        }

        m_textureCache.Clear();
        m_demo = new BasicRigidBodies(this);
        m_renderState = RenderState.m_renderSceneState;
    }

    public Boolean IsInitialized()
    {
        return m_renderInitialized;
    }

    public void SetState(RenderState state)
    {
        m_renderState = state;
    }

    public void Pause()
    {
        SetState(RenderState.m_idleState);
    }

    private nWorld m_world = null;
    private DemoBase m_demo = null;
    private SceneObject m_root = null;
    private SceneCamera m_camera = null;
    private ShaderCache m_shaderCache = null;
    private AssetManager m_assetManager = null;
    private SceneMeshTextureCache m_textureCache = null;

    private int m_screenWidth = 0;
    private int m_screenHeight = 0;
    private float m_timestep = 1.0f / 60.0f;
    private Boolean m_renderInitialized = false;
    private RenderState m_renderState = RenderState.m_idleState;
}