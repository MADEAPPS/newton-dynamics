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

import com.javaNewton.nMatrix;

public class SceneObject
{
    public SceneObject ()
    {
        m_next = null;
        m_prev = null;
        m_parent = null;
        m_lastChild = null;
        m_firstChild = null;

        m_mesh = null;
        m_matrix = new nMatrix();
        m_meshMatrix = new nMatrix();
    }

    public void CleanUp (RenderScene scene)
    {
        if (m_mesh != null)
        {
            m_mesh.CleanUp (scene);
        }

        for (SceneObject child = m_firstChild; child != null; child = child.m_next)
        {
            child.CleanUp (scene);
        }
    }

    public void AttachToParent(SceneObject parent)
    {
        m_parent = parent;
        if (parent.m_firstChild != null)
        {
            m_prev = m_parent.m_lastChild;
            m_parent.m_lastChild.m_next = this;
        }
        else
        {
            m_parent.m_firstChild = this;
        }
        m_parent.m_lastChild = this;
    }

    public nMatrix GetMatrix ()
    {
        return m_matrix;
    }

    public void SetMatrix (nMatrix matrix)
    {
        m_matrix.Set(matrix);
    }

    public nMatrix GetMeshMatrix ()
    {
        return m_meshMatrix;
    }

    public void SetMeshMatrix (nMatrix matrix)
    {
        m_meshMatrix.Set(matrix);
    }

    public SceneMesh GetMesh ()
    {
        return m_mesh;
    }

    public void SetMesh (SceneMesh mesh)
    {
        m_mesh = mesh;
    }

    public void Render (RenderScene scene, nMatrix parentMatrix)
    {
        nMatrix matrix = m_matrix.Mul(parentMatrix);
        if (m_mesh != null)
        {
            nMatrix renderMesh = m_meshMatrix.Mul(matrix);
            m_mesh.Render (scene, renderMesh);
        }

        for (SceneObject child = m_firstChild; child != null; child = child.m_next)
        {
            child.Render (scene, matrix);
        }
    }

    private SceneMesh m_mesh;
    private nMatrix m_matrix;
    private nMatrix m_meshMatrix;

    private SceneObject m_next;
    private SceneObject m_prev;
    private SceneObject m_parent;
    private SceneObject m_lastChild;
    private SceneObject m_firstChild;
}
