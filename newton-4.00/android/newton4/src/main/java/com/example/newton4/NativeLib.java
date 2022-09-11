package com.example.newton4;

public class NativeLib {

    // Used to load the 'newton4' library on application startup.
    static {
        System.loadLibrary("newton4");
    }

    /**
     * A native method that is implemented by the 'newton4' native library,
     * which is packaged with this application.
     */
    public native String stringFromJNI();
}