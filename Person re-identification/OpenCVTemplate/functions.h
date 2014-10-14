
#include <Windows.h>
#include <Kinect.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Safe release for interfaces
template<class Interface>
inline void SAFERELEASE(Interface *& pInterfaceToRelease) {
    if (pInterfaceToRelease != nullptr) {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = nullptr;
    }
}


void processRGBData  (cv::Mat& RGBimage, IColorFrameReader*colorFrameReader); 
void processBodyData (cv::Mat img, IBodyFrameReader* bodyFrameReader1, 
					    ICoordinateMapper* m_pCoordinateMapper, bool& Inview); 
void initializeBodyFeature (double& Feature_Head_height, double& Feature_Torso_ratio,
						      IBodyFrameReader*  bodyFrameReader2);