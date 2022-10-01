package com.example.androidapp;

import android.util.Log;

import com.javaNewton.nMatrix;

import java.nio.FloatBuffer;
import java.nio.ShortBuffer;

public class SceneMesh
{
    public SceneMesh()
    {
        vertexBuffer = null;
        drawListBuffer = null;
    }

    public void Render (nMatrix matrix)
    {
        Log.i("ndNewton", "Render this mesh");
    }


    protected FloatBuffer vertexBuffer;
    protected ShortBuffer drawListBuffer;
}
