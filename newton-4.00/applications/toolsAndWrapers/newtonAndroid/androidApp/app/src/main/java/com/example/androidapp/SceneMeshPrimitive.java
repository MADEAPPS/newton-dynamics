package com.example.androidapp;

import com.javaNewton.nMatrix;
import com.javaNewton.nMeshEffect;
import com.javaNewton.nShapeInstance;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class SceneMeshPrimitive extends SceneMesh
{
    SceneMeshPrimitive(nShapeInstance shapeInstance)
    {
        super();

        nMeshEffect meshEffect = new nMeshEffect(shapeInstance);
        int vertexSizeInBytes = (3 + 3 + 2) * 4;
        int vertexCount = meshEffect.GetVertextCount();
        ByteBuffer bb = ByteBuffer.allocateDirect(vertexCount * vertexSizeInBytes);
        bb.order(ByteOrder.nativeOrder());
        vertexBuffer = bb.asFloatBuffer();
    }

    @Override
    public void Render (nMatrix matrix)
    {

    }
}
