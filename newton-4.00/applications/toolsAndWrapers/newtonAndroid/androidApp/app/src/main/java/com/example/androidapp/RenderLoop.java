package com.example.androidapp;

import android.util.Log;
import android.content.Context;

import com.javaNewton.nBodyNotify;
import com.javaNewton.nMatrix;
import com.javaNewton.nRigidBody;
import com.javaNewton.nShapeBox;
import com.javaNewton.nShapeInstance;
import com.javaNewton.nVector;
import com.javaNewton.nWorld;
import com.newton.nRigidBodyType;

import static android.os.SystemClock.elapsedRealtimeNanos;

public class RenderLoop extends Thread
{
    RenderLoop(Context context)
    {
        m_root = new SceneObject();
        m_glView = new MyGLSurfaceView(context);
        m_glRender = m_glView.GetRenderer();

        m_timestep = 1.0f / 60.0f;
        m_onPause = false;
        m_onTeminate = false;

        // create an instance of the newton engine
        m_world = new nWorld();
        m_world.SetSubSteps(2);
        TestEngine();
    }

    MyGLSurfaceView GetView()
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
        long time_0 = 0;
        long baseTime = elapsedRealtimeNanos();
        long timeStepInNanos = (long) (m_timestep * 1.0e9);
        while (m_onTeminate == false)
        {
            long time_1 = elapsedRealtimeNanos() - baseTime;
            long deltaTime = time_1 - time_0;
            if (m_onPause == false)
            {
                if (deltaTime >= timeStepInNanos)
                {
                    float timestepInSeconds = (float) ((double)deltaTime/1.0e9);
                    DrawFrame(timestepInSeconds);
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

    void DrawFrame(float timestep)
    {
        String text = String.format("RenderLoop fps %f", 1.0f / timestep);
        Log.i("ndNewton", text);

        m_world.Sync();
        m_world.Update(m_timestep);

        nMatrix matrix = new nMatrix();
        m_root.Render(matrix);

        m_glRender.setAngle(m_glRender.getAngle() + 0.1f) ;
        m_glView.requestRender();
    }

    protected void AddFloor()
    {
        nMatrix location = new nMatrix();
        location.Set(3, new nVector(0.0f, -0.5f, 0.0f, 1.0f));

        nRigidBody floor = new nRigidBody(nRigidBodyType.m_dynamic);
        nShapeInstance boxShape = new nShapeInstance(new nShapeBox(200.0f, 1.0f, 200.0f));

        SceneObject floorObject = new SceneObject(m_root);
        SceneMeshPrimitive mesh = new SceneMeshPrimitive(boxShape);
        floorObject.SetMesh(mesh);

        floor.SetNotifyCallback(new BodyNotify(floorObject));
        floor.SetMatrix(location);
        floor.SetCollisionShape(boxShape);
        m_world.AddBody(floor);
    }

    protected void AddBox()
    {
        nMatrix location = new nMatrix();
        location.Set(3, new nVector(0.0f, 5.0f, 0.0f, 1.0f));

        nRigidBody box = new nRigidBody(nRigidBodyType.m_dynamic);
        nShapeInstance boxShape = new nShapeInstance(new nShapeBox(0.5f, 0.5f, 0.5f));

        SceneObject boxObject = new SceneObject(m_root);
        SceneMeshPrimitive mesh = new SceneMeshPrimitive(boxShape);
        boxObject.SetMesh(mesh);

        nBodyNotify notify = new BodyNotify(boxObject);
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

    final private SceneObject m_root;
    final private nWorld m_world;
    final private MyGLSurfaceView m_glView;
    final private MyGLRenderer m_glRender;
    final private float m_timestep;
    private boolean m_onPause;
    private boolean m_onTeminate;
}
