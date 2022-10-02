/*
 * Copyright (C) 2011 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.example.androidapp;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import android.opengl.GLES30;
import android.opengl.Matrix;

import com.javaNewton.nMatrix;

/**
 * A two-dimensional triangle for use as a drawn object in OpenGL ES 2.0.
 */
public class Triangle
{
    private final int m_program;
    private int mPositionHandle;
    private int mColorHandle;
    private int mMVPMatrixHandle;
    private final FloatBuffer vertexBuffer;

    // number of coordinates per vertex in this array
    static final int COORDS_PER_VERTEX = 3;
    static float triangleCoords[] = {
            0.0f,  0.0f,  0.622008459f,
            0.0f,  0.5f, -0.311004243f,
            0.0f, -0.5f, -0.311004243f,
    };
    private final int vertexCount = triangleCoords.length / COORDS_PER_VERTEX;
    private final int vertexStride = COORDS_PER_VERTEX * 4; // 4 bytes per vertex

    float color[] = { 0.63671875f, 0.76953125f, 0.22265625f, 0.0f };

    /**
     * Sets up the drawing object data for use in an OpenGL ES context.
     */
    public Triangle(int shaderProgram)
    {
        m_program = shaderProgram;
        ByteBuffer bb = ByteBuffer.allocateDirect(triangleCoords.length * 4);
        bb.order(ByteOrder.nativeOrder());

        vertexBuffer = bb.asFloatBuffer();
        vertexBuffer.put(triangleCoords);
        vertexBuffer.position(0);
    }

    public void draw(SceneCamera camera)
    {
        // Add program to OpenGL environment
        GLES30.glUseProgram(m_program);

        // get handle to vertex shader's vPosition member
        mPositionHandle = GLES30.glGetAttribLocation(m_program, "vPosition");

        // Enable a handle to the triangle vertices
        GLES30.glEnableVertexAttribArray(mPositionHandle);

        // Prepare the triangle coordinate data
        GLES30.glVertexAttribPointer(
                mPositionHandle, COORDS_PER_VERTEX,
                GLES30.GL_FLOAT, false,
                vertexStride, vertexBuffer);

        // get handle to fragment shader's vColor member
        mColorHandle = GLES30.glGetUniformLocation(m_program, "vColor");

        // Set color for drawing the triangle
        GLES30.glUniform4fv(mColorHandle, 1, color, 0);

        // get handle to shape's transformation matrix
        mMVPMatrixHandle = GLES30.glGetUniformLocation(m_program, "uMVPMatrix");
        RenderScene.checkGlError("glGetUniformLocation");

        m_angle += 0.1f;
        Matrix.setRotateM(mRotationMatrix, 0, m_angle, 1.0f, 0.0f, 0.0f);

        nMatrix rotationMatrix = new nMatrix();
        rotationMatrix.Set(mRotationMatrix);
        nMatrix viewProjectionMatrix = camera.GetMatrix().Mul(camera.GetProjectionMatrix());
        viewProjectionMatrix = rotationMatrix.Mul(viewProjectionMatrix);
        //Matrix.multiplyMM(scratch, 0, mMVPMatrix, 0, mRotationMatrix, 0);
        float[] mvpMatrix = new float[16];
        viewProjectionMatrix.GetFlatArray(mvpMatrix);

        // Apply the projection and view transformation
        GLES30.glUniformMatrix4fv(mMVPMatrixHandle, 1, false, mvpMatrix, 0);
        RenderScene.checkGlError("glUniformMatrix4fv");

        // Draw the triangle
        GLES30.glDrawArrays(GLES30.GL_TRIANGLES, 0, vertexCount);

        // Disable vertex array
        GLES30.glDisableVertexAttribArray(mPositionHandle);
    }

    float m_angle = 0.0f;
    private final float[] mRotationMatrix = new float[16];
}
