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

import android.opengl.GLES30;

import com.javaNewton.nMatrix;
import com.javaNewton.nMeshEffect;
import com.javaNewton.nShapeInstance;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;

public class SceneMeshPrimitive extends SceneMesh
{
    SceneMeshPrimitive(nShapeInstance shapeInstance, RenderScene scene)
    {
        super();

        // get vertex data from mesh and make a vertex buffer for rendering
        int vertexSizeInFloats = (3 + 3 + 2);
        nMeshEffect meshEffect = new nMeshEffect(shapeInstance);

        // get vertex data from newton mesh effect.
        int vertexCount = meshEffect.GetVertextCount();
        float[] vertexData = new float[vertexCount * vertexSizeInFloats];
        meshEffect.GetVertexPosit(vertexData, 0, vertexSizeInFloats);
        meshEffect.GetVertexNormal(vertexData, 3, vertexSizeInFloats);
        meshEffect.GetVertexUV0(vertexData, 6, vertexSizeInFloats);

        // make a gpu formatted vertex buffer.
        int vertexSizeInBytes = vertexSizeInFloats * 4;
        ByteBuffer bb = ByteBuffer.allocateDirect(vertexCount * vertexSizeInBytes);
        bb.order(ByteOrder.nativeOrder());
        FloatBuffer gpuReadyBuffer = bb.asFloatBuffer();
        gpuReadyBuffer.put(vertexData);
        gpuReadyBuffer.position(0);

        // load data to gpu vertex buffer
        IntBuffer vertexBuffer = IntBuffer.allocate(1);
        GLES30.glGenBuffers(1, vertexBuffer);
        m_vertexBuffer = vertexBuffer.get(0);
        GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, m_vertexBuffer);
        GLES30.glBufferData(GLES30.GL_ARRAY_BUFFER, vertexCount * vertexSizeInBytes, gpuReadyBuffer, GLES30.GL_STATIC_DRAW);
        scene.checkGlError("SceneMeshPrimitive");

        // now create a vertex array buffer and set the data layout
        IntBuffer vaoIdBuffer = IntBuffer.allocate(1);
        GLES30.glGenVertexArrays(1, vaoIdBuffer);
        m_vertextArrayBuffer = vaoIdBuffer.get(0);
        GLES30.glBindVertexArray(m_vertextArrayBuffer);

        GLES30.glEnableVertexAttribArray(0);
        GLES30.glVertexAttribPointer(0, 3, GLES30.GL_FLOAT, false, vertexSizeInBytes, 0);

        GLES30.glEnableVertexAttribArray(1);
        GLES30.glVertexAttribPointer(1, 3, GLES30.GL_FLOAT, false, vertexSizeInBytes, 12);

        GLES30.glEnableVertexAttribArray(2);
        GLES30.glVertexAttribPointer(2, 2, GLES30.GL_FLOAT, false, vertexSizeInBytes, 24);
        GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, 0);

        GLES30.glBindVertexArray(0);
        GLES30.glDisableVertexAttribArray(2);
        GLES30.glDisableVertexAttribArray(1);
        GLES30.glDisableVertexAttribArray(0);
        scene.checkGlError("SceneMeshPrimitive");

        // get index data from mesh and make a vertex buffer for rendering
        int indexCount = 0;
        meshEffect.MaterialBegin();
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

        // make a gpu formatted index buffer
        ByteBuffer gpuReadyIndexBuffer = ByteBuffer.allocateDirect(indexCount * 2);
        gpuReadyIndexBuffer.order(ByteOrder.nativeOrder());
        ShortBuffer drawListBuffer = gpuReadyIndexBuffer.asShortBuffer();
        drawListBuffer.put(indexData);
        drawListBuffer.position(0);

        // upload the index buffer data to a gpu index buffer
        IntBuffer indexBuffer = IntBuffer.allocate(1); ;
        GLES30.glGenBuffers(1, indexBuffer);
        m_indexBuffer = indexBuffer.get(0);
        GLES30.glBindBuffer(GLES30.GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
        GLES30.glBufferData(GLES30.GL_ELEMENT_ARRAY_BUFFER, indexCount * 2, gpuReadyIndexBuffer, GLES30.GL_STATIC_DRAW);
        GLES30.glBindBuffer(GLES30.GL_ELEMENT_ARRAY_BUFFER, 0);
        scene.checkGlError("SceneMeshPrimitive");

        m_shader = scene.GetShaderCache().m_directionalDiffuse;

        // get the shader parameters in class variables
        GLES30.glUseProgram(m_shader);
        scene.checkGlError("RenderPrimitive");
        //m_textureLocation = GLES30.glGetUniformLocation(m_shader, "texture");
        //m_transparencyLocation = GLES30.glGetUniformLocation(m_shader, "transparency");
        //m_normalMatrixLocation = GLES30.glGetUniformLocation(m_shader, "normalMatrix");
        //m_projectMatrixLocation = GLES30.glGetUniformLocation(m_shader, "projectionMatrix");
        //m_viewModelMatrixLocation = GLES30.glGetUniformLocation(m_shader, "viewModelMatrix");
        //m_directionalLightDirLocation = GLES30.glGetUniformLocation(m_shader, "directionalLightDir");
        //m_materialAmbientLocation = GLES30.glGetUniformLocation(m_shader, "material_ambient");
        //m_materialDiffuseLocation = GLES30.glGetUniformLocation(m_shader, "material_diffuse");
        //m_materialSpecularLocation = GLES30.glGetUniformLocation(m_shader, "material_specular");
        //GLES30.glUseProgram(0);
    }

    @Override
    public void Render (RenderScene scene, nMatrix matrix)
    {
        //GLES30.glUseProgram(m_shader);
        //m_textureLocation = GLES30.glGetUniformLocation(m_shader, "skinSurface");
        //m_transparencyLocation = GLES30.glGetUniformLocation(m_shader, "transparency");
        //m_normalMatrixLocation = GLES30.glGetUniformLocation(m_shader, "normalMatrix");
        //m_projectMatrixLocation = GLES30.glGetUniformLocation(m_shader, "projectionMatrix");
        //m_viewModelMatrixLocation = GLES30.glGetUniformLocation(m_shader, "viewModelMatrix");
        //m_directionalLightDirLocation = GLES30.glGetUniformLocation(m_shader, "directionalLightDir");
        //m_materialAmbientLocation = GLES30.glGetUniformLocation(m_shader, "material_ambient");
        //m_materialDiffuseLocation = GLES30.glGetUniformLocation(m_shader, "material_diffuse");
        //m_materialSpecularLocation = GLES30.glGetUniformLocation(m_shader, "material_specular");
        //GLES30.glUseProgram(0);
    }

    private int m_textureLocation = -1;
    private int m_transparencyLocation = -1;
    private int m_normalMatrixLocation = -1;
    private int m_projectMatrixLocation = -1;
    private int m_viewModelMatrixLocation = -1;
    private int m_directionalLightDirLocation = -1;
    private int m_materialAmbientLocation = -1;
    private int m_materialDiffuseLocation = -1;
    private int m_materialSpecularLocation = -1;
}
