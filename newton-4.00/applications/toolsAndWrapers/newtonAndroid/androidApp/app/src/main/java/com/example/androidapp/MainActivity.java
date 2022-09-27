package com.example.androidapp;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.view.View;
import android.opengl.GLSurfaceView;

import com.javaNewton.nWorld;
import com.javaNewton.nMatrix;
import com.javaNewton.nVector;
import com.javaNewton.nShapeBox;
import com.javaNewton.nRigidBody;
import com.javaNewton.nBodyNotify;
import com.javaNewton.nShapeInstance;

import com.newton.nRigidBodyType;

public class MainActivity extends AppCompatActivity
{
    // make sure you load the engine dynamic library
    static
    {
        System.loadLibrary("ndNewton");
    }

   private nWorld world;
   private GLSurfaceView mGLView;

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);

		// create an instance of the newton engine
		world = new nWorld();
		world.SetSubSteps(2);
		
		TestEngine();

		// Create a GLSurfaceView instance and set it
		// as the ContentView for this Activity
		mGLView = new MyGLSurfaceView(this);
		setContentView(mGLView);
    }

	@Override
	protected void onPause()
	{
		super.onPause();
		// The following call pauses the rendering thread.
		// If your OpenGL application is memory intensive,
		// you should consider de-allocating objects that
		// consume significant memory here.
		mGLView.onPause();
    }

	@Override
	protected void onResume()
	{
		super.onResume();
		// The following call resumes a paused rendering thread.
		// If you de-allocated graphic objects for onPause()
		// this is a good place to re-allocate them.
		mGLView.onResume();
	}

	@Override
	public void onWindowFocusChanged(boolean hasFocus)
	{
		super.onWindowFocusChanged(hasFocus);
		if (hasFocus) {
			getWindow().getDecorView().setSystemUiVisibility(
					View.SYSTEM_UI_FLAG_LAYOUT_STABLE |
					View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION |
					View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN |
					View.SYSTEM_UI_FLAG_HIDE_NAVIGATION	|
					View.SYSTEM_UI_FLAG_FULLSCREEN);
		}
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
		world.AddBody(floor);
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
		world.AddBody(box);
	}

	protected void TestEngine()
	{
		world.Sync();

		AddFloor();
		AddBox();
		for (int i = 0; i < 100; i++)
		{
			world.Update(1.0f / 60.0f);
			world.Sync();
		}
	}
}