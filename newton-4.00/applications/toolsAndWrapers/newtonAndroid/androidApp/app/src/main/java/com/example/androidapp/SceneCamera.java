package com.example.androidapp;

import android.opengl.GLES30;
import android.opengl.Matrix;

import com.javaNewton.nMatrix;

public class SceneCamera extends SceneObject
{
    SceneCamera()
    {
        super();

        m_projectionMatrix = new nMatrix();
        m_glProjectionMatrix = new float[16];
    }

    public nMatrix GetProjectionMatrix()
    {
        return m_projectionMatrix;
    }

    public void SetProjectionMatrix(int viewportWith, int viewportHeight)
    {
        GLES30.glViewport(0, 0, viewportWith, viewportHeight);

        float ratio = (float) viewportWith / viewportHeight;
        Matrix.frustumM(m_glProjectionMatrix, 0, -ratio, ratio, -1, 1, 3, 7);
        m_projectionMatrix.Set(m_glProjectionMatrix);
    }

    private nMatrix m_projectionMatrix;
    private float[] m_glProjectionMatrix;
}
