/*
 * Copyright (C) 2011 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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

public class MyGLRenderer implements GLSurfaceView.Renderer
{
    public nWorld GetWorld()
    {
        return m_world;
    }

    public float GetTimestep()
    {
        return m_timestep;
    }

    @Override
    public void onSurfaceCreated(GL10 unused, EGLConfig config)
    {
        // Set the background frame color
        GLES30.glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

        m_timestep = 1.0f / 60.0f;
        m_shaderCache = new ShaderCache();

        m_world = new nWorld();
        m_world.SetSubSteps(2);
        m_root = new SceneObject();

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

            nMatrix matrix = new nMatrix();
            m_root.Render(matrix);

            setAngle(getAngle() + 0.1f) ;
            float[] scratch = new float[16];
            // Set the camera position (View matrix)
            Matrix.setLookAtM(mViewMatrix, 0, 0, 0, -3, 0f, 0f, 0f, 0f, 1.0f, 0.0f);

            // Calculate the projection and view transformation
            Matrix.multiplyMM(mMVPMatrix, 0, mProjectionMatrix, 0, mViewMatrix, 0);

            // Draw square
            mSquare.draw(mMVPMatrix);
            Matrix.setRotateM(mRotationMatrix, 0, mAngle, 0, 0, 1.0f);
            Matrix.multiplyMM(scratch, 0, mMVPMatrix, 0, mRotationMatrix, 0);
            mTriangle.draw(scratch);
        }
        else
        {
           GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT | GLES30.GL_DEPTH_BUFFER_BIT);
        }
    }

    @Override
    public void onSurfaceChanged(GL10 unused, int width, int height)
    {
        // Adjust the viewport based on geometry changes,
        // such as screen rotation
        GLES30.glViewport(0, 0, width, height);

        float ratio = (float) width / height;

        // this projection matrix is applied to object coordinates
        // in the onDrawFrame() method
        Matrix.frustumM(mProjectionMatrix, 0, -ratio, ratio, -1, 1, 3, 7);
    }

    public ShaderCache GetShaderCache()
    {
        return m_shaderCache;
    }

    /**
    * Utility method for debugging OpenGL calls. Provide the name of the call
    * just after making it:
    *
    * <pre>
    * mColorHandle = GLES30.glGetUniformLocation(mProgram, "vColor");
    * MyGLRenderer.checkGlError("glGetUniformLocation");</pre>
    *
    * If the operation is not successful, the check throws an error.
    *
    * @param glOperation - Name of the OpenGL call to check.
    */
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

    public float getAngle()
    {
        return mAngle;
    }
    public void setAngle(float angle)
    {
        mAngle = angle;
    }

    private static final String TAG = "MyGLRenderer";

    private nWorld m_world = null;
    private SceneObject m_root = null;
    private ShaderCache m_shaderCache = null;

    private Square mSquare;
    private Triangle mTriangle;

    private Boolean m_renderReady = false;
    private Boolean m_renderInitialized = false;
    private float m_timestep = 1.0f / 60.0f;
    private final float[] mMVPMatrix = new float[16];
    private final float[] mProjectionMatrix = new float[16];
    private final float[] mViewMatrix = new float[16];
    private final float[] mRotationMatrix = new float[16];
    private float mAngle;

}