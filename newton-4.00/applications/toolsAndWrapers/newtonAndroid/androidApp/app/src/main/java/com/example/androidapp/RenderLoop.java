package com.example.androidapp;

import android.util.Log;
import android.content.Context;
import android.opengl.GLSurfaceView;

import com.javaNewton.nBodyNotify;
import com.javaNewton.nMatrix;
import com.javaNewton.nRigidBody;
import com.javaNewton.nShapeBox;
import com.javaNewton.nShapeInstance;
import com.javaNewton.nVector;
import com.javaNewton.nWorld;
import com.newton.nRigidBodyType;

public class RenderLoop extends Thread
{
    private nWorld m_world;
    private GLSurfaceView m_glView;
    private float timestep = 1.0f / 60.0f;
    private boolean m_teminate;

    RenderLoop(Context context)
    {
        m_glView = new MyGLSurfaceView(context);

        m_teminate = false;

        // create an instance of the newton engine
        m_world = new nWorld();
        m_world.SetSubSteps(2);

        TestEngine();
    }

    GLSurfaceView GetView()
    {
        return m_glView;
    }

    @Override
    public void run()
    {
        long time0 = System.currentTimeMillis();
        float timeStepInMs = timestep * 1000.0f;
        while (m_teminate == false)
        {
            long time1 = System.currentTimeMillis();
            float deltaTime = time1 - time0;
            if (deltaTime >= timeStepInMs)
            {
                float fps = 1000.0f / timeStepInMs;
                String text = String.format("RenderLoop fps %f", fps);
                Log.i("ndNewton", text);
                DrawFrame();
                time0 += timeStepInMs;
            }
            yield();
        }
    }

    void DrawFrame()
    {

    }

    protected void AddFloor()
    {
        nMatrix location = new nMatrix();
        location.SetIdentity();
        location.Set(3, new nVector(0.0f, -0.5f, 0.0f, 1.0f));

        nRigidBody floor = new nRigidBody(nRigidBodyType.m_dynamic);
        nShapeInstance boxShape = new nShapeInstance(new nShapeBox(200.0f, 1.0f, 200.0f));

        floor.SetNotifyCallback(new nBodyNotify());
        floor.SetMatrix(location);
        floor.SetCollisionShape(boxShape);
        m_world.AddBody(floor);
    }

    protected void AddBox()
    {
        nMatrix location = new nMatrix();
        location.SetIdentity();
        location.Set(3, new nVector(0.0f, 5.0f, 0.0f, 1.0f));

        nRigidBody box = new nRigidBody(nRigidBodyType.m_dynamic);
        nShapeInstance boxShape = new nShapeInstance(new nShapeBox(0.5f, 0.5f, 0.5f));
        nBodyNotify notify = new nBodyNotify();
        notify.SetGravity(new nVector(0.0f, -10.0f, 0.0f, 0.0f));

        box.SetNotifyCallback(notify);
        box.SetMatrix(location);
        box.SetCollisionShape(boxShape);
        box.SetMassMatrix(1.0f, boxShape);
        m_world.AddBody(box);
    }

    protected void TestEngine()
    {
        m_world.Sync();

        AddFloor();
        AddBox();
        for (int i = 0; i < 100; i++)
        {
            m_world.Update(timestep);
            m_world.Sync();
        }
    }
}
