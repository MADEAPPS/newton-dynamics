package com.example.androidapp;

import android.util.Log;
import android.content.Context;
import com.example.androidapp.MyGLRenderer;
import com.example.androidapp.MyGLSurfaceView;

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
    final private nWorld m_world;
    final private MyGLSurfaceView m_glView;
    final private MyGLRenderer m_glRender;
    final private float m_timestep;
    final private boolean m_teminate;

    RenderLoop(Context context)
    {
        m_glView = new MyGLSurfaceView(context);
        m_glRender = m_glView.GetRenderer();

        m_timestep = 1.0f / 60.0f;
        m_teminate = false;

        // create an instance of the newton engine
        m_world = new nWorld();
        m_world.SetSubSteps(2);
        TestEngine();
    }

    MyGLSurfaceView GetView()
    {
        return m_glView;
    }

    @Override
    final public void run()
    {

        double time_0 = 0.0;
        long baseTime = System.currentTimeMillis();
        double timeStepInMs = m_timestep * 1000.0f;
        while (m_teminate == false)
        {
            double time_1 = (System.currentTimeMillis() - baseTime);
            double deltaTime = time_1 - time_0;
            if (deltaTime < 0.0)
            {
                Log.i("ndNewton", "xxx");
            }
            if (deltaTime >= timeStepInMs)
            {
                float fps = (float)(1000.0 / deltaTime);
                String text = String.format("RenderLoop fps %f", fps);
                Log.i("ndNewton", text);
                DrawFrame();
                time_0 = time_0 + timeStepInMs;
                while ((time_0 + timeStepInMs) < time_1)
                {
                    Log.i("ndNewton", "skip frame");
                    time_0 = time_0 + timeStepInMs;
                }
            }
            yield();
        }
    }

    void DrawFrame()
    {
        m_world.Sync();
        m_world.Update(m_timestep);

        m_glRender.setAngle(m_glRender.getAngle() + 0.1f) ;
        m_glView.requestRender();
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
    }
}
