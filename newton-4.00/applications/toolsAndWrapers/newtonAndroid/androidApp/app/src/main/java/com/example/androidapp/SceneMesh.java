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
        m_positHandle = 0;
        m_normalHandle = 0;
        m_uvlHandle = 0;

        m_vertexBuffer = null;
        m_drawListBuffer = null;
        m_segments = new ArrayList<SceneMeshSegment>();
    }

    void AddSegment(SceneMeshSegment segment)
    {
        m_segments.add (segment);
    }

    public void Render (nMatrix matrix)
    {
        Log.i("ndNewton", "Render this mesh");
    }

    protected int m_shader;
    protected int m_positHandle;
    protected int m_normalHandle;
    protected int m_uvlHandle;
    protected FloatBuffer m_vertexBuffer;
    protected ShortBuffer m_drawListBuffer;
    ArrayList<SceneMeshSegment> m_segments;
}
