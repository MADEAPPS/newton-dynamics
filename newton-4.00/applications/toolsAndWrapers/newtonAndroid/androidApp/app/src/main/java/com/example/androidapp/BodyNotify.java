package com.example.androidapp;


import android.util.Log;

import com.javaNewton.nMatrix;
import com.newton.ndMatrixGlue;
import com.javaNewton.nBodyNotify;

public class BodyNotify extends nBodyNotify
{
    BodyNotify(SceneObject object)
    {
        super();
        m_object = object;
    }

    public void OnTransform(ndMatrixGlue matrix)
    {
        nMatrix objectMatrix = new nMatrix(matrix);
        m_object.SetMatrix(objectMatrix);
    }

    SceneObject m_object;
}
