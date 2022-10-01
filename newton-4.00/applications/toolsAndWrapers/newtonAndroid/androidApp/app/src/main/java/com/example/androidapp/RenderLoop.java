package com.example.androidapp;

import android.util.Log;
import android.content.Context;

import static android.os.SystemClock.elapsedRealtimeNanos;

public class RenderLoop extends Thread
{
    RenderLoop(Context context)
    {
        m_demo = null;
        m_onPause = false;
        m_onTeminate = false;
        m_glView = new MyGLSurfaceView(context);
        m_glRender = m_glView.GetRenderer();
    }

    public MyGLSurfaceView GetView()
    {
        return m_glView;
    }

    public void OnTerminate()
    {
        m_onTeminate = true;
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
        while (m_glRender.GetShaderCache() == null)
        {
            yield();
        }

        LoadDemo();

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
                    float timestepInSeconds = (float) ((double)deltaTime/1.0e9);
                    //DrawFrame(timestepInSeconds);
                    String text = String.format("RenderLoop fps %f", 1.0f / timestepInSeconds);
                    Log.i("ndNewton", text);

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
    }

    public void LoadDemo()
    {
        m_glRender.GetWorld().Sync();
        if (m_demo != null)
        {
            m_glRender.Pause();
            m_demo.CleanUp(m_glRender);
        }
        m_demo = new DemosBase_BasicRigidBodies(m_glRender);
        m_glRender.SetReady();
    }

    private DemosBase m_demo;
    final private MyGLSurfaceView m_glView;
    final private MyGLRenderer m_glRender;

    private boolean m_onPause;
    private boolean m_onTeminate;
}
