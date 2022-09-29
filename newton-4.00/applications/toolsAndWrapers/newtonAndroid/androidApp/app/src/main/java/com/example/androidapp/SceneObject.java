package com.example.androidapp;

import com.javaNewton.nMatrix;

public class SceneObject
{
    SceneObject ()
    {
        m_mesh = null;
        m_next = null;
        m_prev = null;
        m_parent = null;
        m_firstChild = null;

        m_matrix = new nMatrix();
        m_meshMatrix = new nMatrix();
    }

    SceneObject (SceneObject parent)
    {
        m_mesh = null;
        m_parent = parent;
        m_next = parent.m_firstChild;
        if (parent.m_firstChild != null)
        {
            parent.m_firstChild.m_prev = this;
        }
        m_prev = null;
        m_firstChild = null;
        parent.m_firstChild = this;

        m_matrix = new nMatrix();
        m_meshMatrix = new nMatrix();
    }

    public void SetMesh (SceneMesh mesh)
    {
        m_mesh = mesh;
    }

    public void SetMatrix (nMatrix matrix)
    {
        m_matrix.Set(matrix);
    }

    void Render (nMatrix parentMatrix)
    {
        nMatrix matrix = m_matrix.Mul(parentMatrix);
        if (m_mesh != null)
        {
            nMatrix renderMesh = m_meshMatrix.Mul(matrix);
            m_mesh.Render (renderMesh);
        }

        for (SceneObject child = m_firstChild; child != null; child = child.m_next)
        {
            child.Render (matrix);
        }
    }

    private nMatrix m_matrix;
    private nMatrix m_meshMatrix;

    private SceneMesh m_mesh;
    private SceneObject m_next;
    private SceneObject m_prev;
    private SceneObject m_parent;
    private SceneObject m_firstChild;

}
