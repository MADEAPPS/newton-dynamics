package com.javaNewton;

import android.util.Log;

import com.newton.ndMatrixGlue;
import com.newton.ndBodyNotifyGlue;
import com.example.androidapp.SceneObject;

public class nBodyNotify extends ndBodyNotifyGlue
{
    public nBodyNotify(SceneObject object)
    {
        super();
        m_object = object;
    }

    public void OnTransform(ndMatrixGlue matrix)
    {
        Log.i("ndNewton", "OnTransform!!!");
    }

    SceneObject m_object;
}
