all: svo/toolchain svo/lib/libsvo.so libs/x86/libar_jni.so
	ant debug

libs/x86/libar_jni.so: jni/ar_jni.cpp jni/ar_jni.h
	~/android-sdk-linux/ndk-bundle/ndk-build -B

svo/toolchain:
	mkdir -p svo/toolchain
	~/android-sdk-linux/ndk-bundle/build/tools/make-standalone-toolchain.sh \
	  --toolchain=x86-svo --force --platform=android-21 --install-dir='$@'

svo/lib/libsvo.so: svo/toolchain
	cd svo && cmake . && make -j6

.PHONY: svo/lib/libsvo.so
