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
        //Log.i("ndNewton", "OnTransform!!!");
        m_object.SetMatrix(matrix);
    }

    SceneObject m_object;
}
