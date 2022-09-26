package com.javaNewton;

import android.util.Log;

import com.newton.ndMatrixGlue;
import com.newton.ndBodyNotifyGlue;

public class nBodyNotify extends ndBodyNotifyGlue
{

    public void OnTransform(ndMatrixGlue matrix)
    {
        Log.i("ndNewton", "OnTransform!!!");
    }
}
