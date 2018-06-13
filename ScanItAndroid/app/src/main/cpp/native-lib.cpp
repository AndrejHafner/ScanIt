#include <jni.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "ImageProcessing.h"

using namespace std;
using namespace cv;

extern "C"
{

jboolean JNICALL Java_com_scanit_likewise_scanitandroid_MainActivity_salt(JNIEnv *env, jobject instance,
                                                                           jlong matAddrGray,
                                                                           jlong extractAddr, jboolean extract) {
    Mat& inputImage = *(Mat*)matAddrGray;
    Mat& extr = *(Mat*)extractAddr;

    bool res = getDocumentOutline(inputImage,extr,(bool) extract);
//    testFunc(inputImage);
    if((bool) extract && res)
    {
        return JNI_TRUE;
    } else
        return JNI_FALSE;
}

}

