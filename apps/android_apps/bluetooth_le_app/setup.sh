#!/usr/bin/env bash

ANDROID_APP_PATH=`pwd`
WISELIB_PATH=`readlink -m ${ANDROID_APP_PATH}/../../`
WISELIB_ANDROID=${WISELIB_PATH}/wiselib.testing/external_interface/android
ANDROID_TOOLS=${HOME}/work/android

JNI_PATH=${ANDROID_APP_PATH}/jni


debug_msg ()
{
 echo -e "\e[00;31m ${1} \e[00m"
}


cleanup ()
{
  debug_msg "Cleaning up"
  rm -rf ${ANDROID_TOOLS}
}


setup_android()
{
  debug_msg "Installing the toolchain for building wiselib applications for Android"

  sudo apt-get install -qq build-essential openjdk-6-jre openjdk-6-jdk ant
  
  mkdir -p ${ANDROID_TOOLS}

  debug_msg "Downloading Android SDK & NDK"
  wget -q http://dl.google.com/android/android-sdk_r20-linux.tgz -O ${ANDROID_TOOLS}/sdk.tgz
  wget -q http://dl.google.com/android/ndk/android-ndk-r8-linux-x86.tar.bz2 -O ${ANDROID_TOOLS}/ndk.tar.bz2

  debug_msg "Unpacking Android SDK & NDK"
  tar xzf ${ANDROID_TOOLS}/sdk.tgz -C ${ANDROID_TOOLS}
  tar xjf ${ANDROID_TOOLS}/ndk.tar.bz2 -C ${ANDROID_TOOLS}

  debug_msg "Add SDK and NDK to PATH"
   
  echo -e "\n# Configure Android environment \n" >> ~/.bashrc

  echo -e "export ANDROID_SDK=${ANDROID_TOOLS}/android-sdk-linux" >> ~/.bashrc
  echo -e "export ANDROID_NDK=${ANDROID_TOOLS}/android-ndk-r8" >> ~/.bashrc

  echo -e 'export PATH=$PATH:$ANDROID_SDK/tools:$ANDROID_SDK/platform-tools:$ANDROID_NDK' >> ~/.bashrc

  source ~/.bashrc

  debug_msg "Install SDK for Android 2.3.*"
  compatible_sdk=`android list sdk | grep "SDK Platform Android .* API 10"`
  get_sdk_number=`echo ${compatible_sdk} |grep -Eo "^[[:digit:]]+"`

  platform_tools=`android list sdk | grep "Android SDK Platform-tools"`
  platform_number=`echo ${platform_tools} |grep -Eo "^[[:digit:]]+"`

  android update sdk -s -u -t ${get_sdk_number},${platform_number}
}


configure_project () 
{
   debug_msg "Configure Android application"
   echo ${WISELIB_PATH}
   echo ${JNI_PATH}
   sed -i "s#^WISELIB_PATH := .*#WISELIB_PATH := ${WISELIB_PATH}#" ${JNI_PATH}/Android.mk

   cd jni/
   ndk-build
   cd ..

   sed -i "s#<location>.*</location>#<location>${WISELIB_ANDROID}</location>#" .project

   sed -i "s#^sdk.dir=.*#sdk.dir=${ANDROID_TOOLS}/android-sdk-linux#" local.properties

   sed -i "s#^source\.dir=.*#source\.dir=src;${WISELIB_ANDROID}#" ant.properties
}


# Main

cleanup
setup_android
configure_project
