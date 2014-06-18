#include "android_onload.h"

extern "C" {
   JNIEXPORT jint JNI_OnLoad ( JavaVM* jVM, void* reserved )
   {

      __GLOBAL_JAVA_VM__ = jVM;

      JNIEnv* env;

      if ( jVM->GetEnv( ( void** ) &env, JNI_VERSION_1_6 ) != JNI_OK )
      {
         __android_log_print( ANDROID_LOG_DEBUG, "WiselibDebug: ", "JNI_OnLoad failed" );
         return -1;
      }

      __android_log_print( ANDROID_LOG_DEBUG, "WiselibDebug: ", "JNI_OnLoad success" );
      return JNI_VERSION_1_6;
   }
}