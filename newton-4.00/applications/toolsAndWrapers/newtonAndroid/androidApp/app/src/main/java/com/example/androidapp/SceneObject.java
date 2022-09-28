package com.example.androidapp;

public class SceneObject
{
    SceneObject ()
    {
        m_mesh = null;
        m_next = null;
        m_prev = null;
        m_parent = null;
        m_firstChild = null;
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
    }

    void SetMesh (SceneMesh mesh)
    {
        m_mesh = mesh;
    }

    void Render (Matrix4 parentMatrix)
    {
        Matrix4 matrix = m_matrix.Mul(parentMatrix);
        if (m_mesh != null)
        {
            Matrix4 renderMesh = m_meshMatrix.Mul(matrix);
            m_mesh.Render (renderMesh);
        }

        for (SceneObject child = m_firstChild; child != null; child = child.m_next)
        {
            Render (matrix);
        }
    }

    Matrix4 m_matrix;
    Matrix4 m_meshMatrix;

    private SceneMesh m_mesh;
    private SceneObject m_next;
    private SceneObject m_prev;
    private SceneObject m_parent;
    private SceneObject m_firstChild;

}
