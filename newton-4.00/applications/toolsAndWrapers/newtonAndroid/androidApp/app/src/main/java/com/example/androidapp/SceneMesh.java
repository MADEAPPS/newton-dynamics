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
        vertexBuffer = null;
        drawListBuffer = null;
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

    protected FloatBuffer vertexBuffer;
    protected ShortBuffer drawListBuffer;
    ArrayList<SceneMeshSegment> m_segments;
}
