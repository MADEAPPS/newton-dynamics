package com.example.androidapp;

import com.javaNewton.nMatrix;
import com.javaNewton.nMeshEffect;
import com.javaNewton.nShapeInstance;

public class SceneMeshPrimitive extends SceneMesh
{
    SceneMeshPrimitive(nShapeInstance shapeInstance)
    {
        super();

        nMeshEffect meshEffect = new nMeshEffect(shapeInstance);
    }

    @Override
    public void Render (nMatrix matrix)
    {

    }
}
