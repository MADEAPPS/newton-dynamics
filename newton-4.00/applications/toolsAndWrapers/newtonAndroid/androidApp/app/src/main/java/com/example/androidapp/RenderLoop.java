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

import android.util.Log;
import android.content.Context;

import static android.os.SystemClock.elapsedRealtimeNanos;

public class RenderLoop extends Thread
{
    RenderLoop(Context context)
    {
        m_onPause = false;
        m_onTeminate = false;
        m_glView = new SurfaceView(context);
        m_glRender = m_glView.GetRenderer();
    }

    public SurfaceView GetView()
    {
        return m_glView;
    }

    public void OnTerminate()
    {
        m_onTeminate = true;
        while (m_onTeminate)
        {
            yield();
        }
        try
        {
            join();
        }
        catch(Exception ex)
        {
            System.out.println("Exception has " + "been caught" + ex);
        }
    }

    public void OnPause()
    {
        m_onPause = true;
        m_glView.onPause();
    }

    public void OnResume()
    {
        m_onPause = false;
        m_glView.onResume();
    }

    @Override
    final public void run()
    {
        while (!m_glRender.IsInitialized())
        {
            yield();
        }

        long time_0 = 0;
        long baseTime = elapsedRealtimeNanos();
        long timeStepInNanos = (long) (m_glRender.GetTimestep() * 1.0e9);
        while (!m_onTeminate)
        {
            if (!m_onPause)
            {
                long time_1 = elapsedRealtimeNanos() - baseTime;
                long deltaTime = time_1 - time_0;
                if (deltaTime >= timeStepInNanos)
                {
                    m_glView.requestRender();
                    time_0 = time_0 + timeStepInNanos;
                    if (deltaTime > (4 * timeStepInNanos))
                    {
                        Log.i("ndNewton", "skip frame");
                        long skipFrames = timeStepInNanos * ((deltaTime / timeStepInNanos) - 1);
                        time_0 = time_0 + skipFrames;
                    }
                }
            }
            yield();
        }
        m_glRender.DestroyScene();
        m_onTeminate = false;
    }

    final private SurfaceView m_glView;
    final private RenderScene m_glRender;

    private boolean m_onPause;
    private boolean m_onTeminate;
}
