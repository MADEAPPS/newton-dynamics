package com.example.androidapp;

import android.opengl.GLES30;
import android.opengl.Matrix;

import com.javaNewton.nMatrix;
import com.javaNewton.nVector;

public class SceneCamera extends SceneObject
{
    SceneCamera()
    {
        super();

        m_projectionMatrix = new nMatrix();
        m_glViewMatrix = new float[16];
        m_glProjectionMatrix = new float[16];
    }

    void SetLookAtMatrix (nVector origin, nVector front, nVector up)
    {
        nMatrix result = new nMatrix();

        nVector zAxis = front.Scale (-1.0f);
        zAxis.m_data[3] = 0.0f;
        zAxis = zAxis.Normalize();

        nVector xAxis = up.CrossProduct(zAxis);
        xAxis.m_data[3] = 0.0f;
        xAxis = xAxis.Normalize();
        nVector YAxis = zAxis.CrossProduct(xAxis);

        result.SetRow(0, xAxis);
        result.SetRow(1, YAxis);
        result.SetRow(2, zAxis);
        result = result.Inverxe();

        nVector negEye = origin.Scale (-1.0f);
        negEye.m_data[3] = 1.0f;
        result.SetRow(3, result.TransformVector(negEye));

        GetMatrix().Set(result);
        result.GetFlatArray(m_glViewMatrix);
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

    private float[] m_glViewMatrix;
    private float[] m_glProjectionMatrix;

}
