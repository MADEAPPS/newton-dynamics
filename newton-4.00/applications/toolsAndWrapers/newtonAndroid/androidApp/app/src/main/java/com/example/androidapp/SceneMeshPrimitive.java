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

        int vertexSizeInFloats = (3 + 3 + 2);
        nMeshEffect meshEffect = new nMeshEffect(shapeInstance);

        int vertexCount = meshEffect.GetVertextCount();
        float[] floatData = new float[vertexCount * vertexSizeInFloats];
        meshEffect.GetVertexPosit(floatData, 0, vertexSizeInFloats);
        meshEffect.GetVertexNormal(floatData, 3, vertexSizeInFloats);
        meshEffect.GetVertexUV0(floatData, 6, vertexSizeInFloats);

        int vertexSizeInBytes = vertexSizeInFloats * 4;
        ByteBuffer bb = ByteBuffer.allocateDirect(vertexCount * vertexSizeInBytes);
        bb.order(ByteOrder.nativeOrder());
        vertexBuffer = bb.asFloatBuffer();
        vertexBuffer = vertexBuffer.put(floatData);
        vertexBuffer.position(0);
    }

    @Override
    public void Render (nMatrix matrix)
    {

    }
}
