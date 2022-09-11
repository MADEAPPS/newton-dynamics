#include <jni.h>
#include <string>

#include <sdk/dNewton/ndNewton.h>

extern "C" JNIEXPORT jstring JNICALL
Java_com_example_newton4_NativeLib_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {
    ndNewton* xxx = new ndNewton;
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}