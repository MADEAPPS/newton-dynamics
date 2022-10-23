package com.example.androidapp;

import java.nio.IntBuffer;
import android.opengl.GLES30;
import android.opengl.Matrix;

import com.javaNewton.nMatrix;
import com.javaNewton.nVector;

public class SceneCamera extends SceneObject
{
    SceneCamera()
    {
        super();

        m_frontPlane = 0.01f;
        m_backPlane = 1000.0f;
        m_fov = 60.0f * 3.1416f/ 180.0f;

        m_viewPort = IntBuffer.allocate(4);

        m_viewMatrix = new nMatrix();
        m_projectionMatrix = new nMatrix();
    }

    public nMatrix GetViewMatrix()
    {
        return m_viewMatrix;
    }

    public nMatrix GetProjectionMatrix()
    {
        return m_projectionMatrix;
    }

    public void SetViewMatrix(int screenWidth, int screenHeight)
    {
        GLES30.glViewport(0, 0, screenWidth, screenHeight);
        GLES30.glGetIntegerv(GLES30.GL_VIEWPORT, m_viewPort);

        // calculate projection matrix
        float ratio = (float) screenWidth / (float)screenHeight;
        float y = m_frontPlane * (float)Math.tan(m_fov * 0.5f);
        float x = y * ratio;
        float left = -x;
        float right = x;
        float top = y;
        float bottom = -y;
        SetProjectionMatrix (left, right, bottom, top);

        // calculate view matrix
        nMatrix matrix = GetMatrix ();
        nVector up = new nVector(matrix.GetRow(1));
        nVector target = new nVector(matrix.GetPosition().Add(matrix.GetRow(0)));
        nVector origin = new nVector(matrix.GetPosition());
        SetLookAtMatrix (origin, target, up);
    }

    private void SetProjectionMatrix (float left, float right, float bottom, float top)
    {
        nVector axis0 = new nVector(2.0f * m_frontPlane / (right - left), 0.0f, 0.0f, 0.0f);
        nVector axis1 = new nVector(0.0f, 2.0f * m_frontPlane / (top - bottom), 0.0f, 0.0f);
        nVector axis2 = new nVector((right + left) / (right - left), (top + bottom) / (top - bottom), -(m_backPlane + m_frontPlane) / (m_backPlane - m_frontPlane), -1.0f);
        nVector axis3 = new nVector(0.0f, 0.0f, -2.0f * (m_backPlane * m_frontPlane) / (m_backPlane - m_frontPlane), 0.0f);

        m_projectionMatrix.SetRow(0, axis0);
        m_projectionMatrix.SetRow(1, axis1);
        m_projectionMatrix.SetRow(2, axis2);
        m_projectionMatrix.SetRow(3, axis3);
    }

    private void SetLookAtMatrix (nVector eyepoint, nVector eyepointTarget, nVector up)
    {
        nMatrix result = new nMatrix();

        nVector zAxis = eyepoint.Sub(eyepointTarget);
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

        nVector negEye = eyepoint.Scale (-1.0f);
        negEye.m_data[3] = 1.0f;
        result.SetRow(3, result.TransformVector(negEye));
        m_viewMatrix.Set(result);
    }

    private nMatrix m_viewMatrix;
    private nMatrix m_projectionMatrix;
    private IntBuffer m_viewPort;
    private float m_fov;
    private float m_backPlane;
    private float m_frontPlane;
}
