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

        // get vertex data from mesh and make a vertex buffer for rendering
        int vertexSizeInFloats = (3 + 3 + 2);
        nMeshEffect meshEffect = new nMeshEffect(shapeInstance);

        int vertexCount = meshEffect.GetVertextCount();
        float[] vertexData = new float[vertexCount * vertexSizeInFloats];
        meshEffect.GetVertexPosit(vertexData, 0, vertexSizeInFloats);
        meshEffect.GetVertexNormal(vertexData, 3, vertexSizeInFloats);
        meshEffect.GetVertexUV0(vertexData, 6, vertexSizeInFloats);

        int vertexSizeInBytes = vertexSizeInFloats * 4;
        ByteBuffer bb = ByteBuffer.allocateDirect(vertexCount * vertexSizeInBytes);
        bb.order(ByteOrder.nativeOrder());
        vertexBuffer = bb.asFloatBuffer();
        vertexBuffer.put(vertexData);
        vertexBuffer.position(0);

        // get index data from mesh and make a vertex buffer for rendering
        meshEffect.MaterialBegin();

        int indexCount = 0;
        for (int index = meshEffect.GetFirstMaterial(); index != -1; index = meshEffect.GetNextMaterial(index))
        {
            indexCount += meshEffect.GetMaterialIndexCount(index);
        }
        short[] indexData = new short[indexCount];

        int offset = 0;
        for (int index = meshEffect.GetFirstMaterial(); index != -1; index = meshEffect.GetNextMaterial(index))
        {
            meshEffect.GetMaterialGetIndexStream(index, indexData, offset);
            int segIndexCount = meshEffect.GetMaterialIndexCount(index);

            SceneMeshSegment segment = new SceneMeshSegment(offset, segIndexCount);
            AddSegment(segment);

            offset += segIndexCount;
        }
        meshEffect.MaterialEnd();

        ByteBuffer dlb = ByteBuffer.allocateDirect(indexCount * 2);
        dlb.order(ByteOrder.nativeOrder());
        drawListBuffer = dlb.asShortBuffer();
        drawListBuffer.put(indexData);
        drawListBuffer.position(0);

    }

    @Override
    public void Render (nMatrix matrix)
    {

    }
}
