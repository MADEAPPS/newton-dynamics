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

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import android.opengl.GLES30;
import android.opengl.GLSurfaceView;
import android.opengl.Matrix;
import android.util.Log;

import com.javaNewton.nMatrix;
import com.javaNewton.nWorld;

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

    public SceneCamera GetCamera()
    {
        return m_camera;
    }

    @Override
    public void onSurfaceCreated(GL10 unused, EGLConfig config)
    {
        // Set the background frame color
        GLES30.glCullFace (GLES30.GL_BACK);
        GLES30.glFrontFace (GLES30.GL_CCW);
        GLES30.glEnable (GLES30.GL_CULL_FACE);
        GLES30.glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

        m_timestep = 1.0f / 60.0f;
        m_shaderCache = new ShaderCache();

        m_world = new nWorld();
        m_world.SetSubSteps(2);
        m_root = new SceneObject();
        m_camera = new SceneCamera();

        mSquare   = new Square(m_shaderCache.m_solidColor);
        mTriangle = new Triangle(m_shaderCache.m_solidColor);

        m_renderInitialized = true;
    }

    void AddSceneObject(SceneObject object)
    {
        object.AttachToParent(m_root);
    }

    @Override
    public void onDrawFrame(GL10 unused)
    {
        if (m_renderReady)
        {
            m_world.Sync();
            m_world.Update(m_timestep);

            // Draw background color
            GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT | GLES30.GL_DEPTH_BUFFER_BIT);

            mSquare.draw(m_camera);
            mTriangle.draw(m_camera);

            nMatrix matrix = new nMatrix();
            m_root.Render(matrix);
        }
        else
        {
           GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT | GLES30.GL_DEPTH_BUFFER_BIT);
        }
    }

    @Override
    public void onSurfaceChanged(GL10 unused, int width, int height)
    {
        m_camera.SetProjectionMatrix(width, height);
    }

    public ShaderCache GetShaderCache()
    {
        return m_shaderCache;
    }

    public static void checkGlError(String glOperation)
    {
        int error;
        while ((error = GLES30.glGetError()) != GLES30.GL_NO_ERROR) {
            Log.e(TAG, glOperation + ": glError " + error);
            throw new RuntimeException(glOperation + ": glError " + error);
        }
    }

    public Boolean IsInitialized()
    {
        return m_renderInitialized;
    }

    public void SetReady()
    {
        m_renderReady = true;
    }

    public void Pause()
    {
        m_renderReady = false;
    }

    private static final String TAG = "MyGLRenderer";

    private nWorld m_world = null;
    private SceneObject m_root = null;
    private SceneCamera m_camera = null;
    private ShaderCache m_shaderCache = null;

    private Boolean m_renderReady = false;
    private Boolean m_renderInitialized = false;
    private float m_timestep = 1.0f / 60.0f;

    private Square mSquare;
    private Triangle mTriangle;
}