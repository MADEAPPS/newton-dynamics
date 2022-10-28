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

import android.util.Log;
import android.opengl.GLES30;
import com.javaNewton.nMatrix;

import java.util.ArrayList;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;
import java.util.List;

public class SceneMesh
{
    public SceneMesh()
    {
        m_shader = 0;
        m_indexBuffer = 0;
        m_vertexBuffer = 0;
        m_vertextArrayBuffer = 0;
        m_segments = new ArrayList<SceneMeshSegment>();
    }

    //public void finalize()
    public void CleanUp (RenderScene scene)
    {
        int[] buffer = new int[1];
        if (m_indexBuffer != 0)
        {
            buffer[0] = m_indexBuffer;
            GLES30.glDeleteBuffers(1, buffer, 0);
        }

        if (m_vertexBuffer != 0)
        {
            buffer[0] = m_vertexBuffer;
            GLES30.glDeleteBuffers(1, buffer, 0);
        }

        if (m_vertextArrayBuffer != 0)
        {
            buffer[0] = m_vertextArrayBuffer;
            GLES30.glDeleteVertexArrays(1, buffer, 0);
        }
        m_segments.clear();
    }

    void AddSegment(SceneMeshSegment segment)
    {
        m_segments.add (segment);
    }

    public void Render (RenderScene scene, nMatrix matrix)
    {
        Log.i("ndNewton", "Render this mesh");
    }

    protected int m_shader;
    protected int m_indexBuffer;
    protected int m_vertexBuffer;
    protected int m_vertextArrayBuffer;
    ArrayList<SceneMeshSegment> m_segments;
}
