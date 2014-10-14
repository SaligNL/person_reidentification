
#include <iostream>
#include <sstream>
#include <vector>
#include <Windows.h>
#include <Kinect.h>
#include "functions.h"

inline void CHECKERROR(HRESULT n) {
    if (!SUCCEEDED(n)) {
        std::stringstream ss;
        ss << "ERROR " << std::hex << n << std::endl;
        std::cin.ignore(); 
        std::cin.get();
        throw std::runtime_error(ss.str().c_str());
    }
}

int main(int argc, char** argv) {

	IColorFrameReader* colorFrameReader = nullptr;  // color reader
	IBodyFrameReader*  bodyFrameReader1 = nullptr;  // body reader1
	IBodyFrameReader*  bodyFrameReader2 = nullptr;  // body reader2
	ICoordinateMapper* m_pCoordinateMapper;

    HRESULT hr;
    IKinectSensor* kinectSensor = nullptr;     // kinect sensor

    // initialize Kinect Sensor
    hr = GetDefaultKinectSensor(&kinectSensor);
    if (FAILED(hr) || !kinectSensor) {
        std::cout << "ERROR hr=" << hr << "; sensor=" << kinectSensor << std::endl;
        return -1;
    }
    CHECKERROR(kinectSensor->Open());

    // initialize color frame reader
    IColorFrameSource* colorFrameSource = nullptr; // color source
    CHECKERROR(kinectSensor->get_ColorFrameSource(&colorFrameSource));
    CHECKERROR(colorFrameSource->OpenReader(&colorFrameReader));
    SAFERELEASE(colorFrameSource);

	// initialize body frame reader
	IBodyFrameSource* bodyFrameSource = nullptr;  // body source

	CHECKERROR(kinectSensor->get_BodyFrameSource(&bodyFrameSource));
	CHECKERROR(bodyFrameSource->OpenReader(&bodyFrameReader1));
	CHECKERROR(bodyFrameSource->OpenReader(&bodyFrameReader2));
	CHECKERROR(kinectSensor->get_CoordinateMapper(&m_pCoordinateMapper));
	SAFERELEASE(bodyFrameSource);

	cv::Mat RGBimage; 
	double I_Head_height,  I_Torso_ratio; 
	double Re_Head_height, Re_Torso_ratio; 

	int Number_frame = 0; 
	bool Initialize = true;  
	bool Inview = false;  
	bool Identify_start = false;  
	bool Identify_Person = false; 

    while ((colorFrameReader)&&(bodyFrameReader1)){
		Number_frame++; 
		// Obtain RGB image 
        processRGBData(RGBimage, colorFrameReader);
		// Obtain skeleton image 
		processBodyData(RGBimage, bodyFrameReader1, m_pCoordinateMapper, Inview);

		// Initialize the skeleton feature of tracked person (single person)    
		if((Initialize)&&(Inview)&&(Number_frame>50))
		{
		   initializeBodyFeature(I_Head_height, I_Torso_ratio, bodyFrameReader2); 
		   Initialize = false;
		}

		// Obtain the skeleton feature of new entered person (single person) and determine 
		// if it is the same person or not . If it is the same person "Identify_Person = true", otherwise 
		// "Identify_Person = false". 
		if((Identify_start)&&(Inview)&&(Number_frame>50))
		{
		   initializeBodyFeature(Re_Head_height, Re_Torso_ratio, bodyFrameReader2); 
		   Identify_start = false;
		   double Diff_head = abs(Re_Head_height - I_Head_height); 
		   double Diff_ratio = abs(Re_Torso_ratio - I_Torso_ratio); 
		   std::cout << Diff_head << " "<< Diff_ratio << std::endl; 
		   if ((Diff_head < 0.05)&&(Diff_ratio < 0.05))  
		   {
			  Identify_Person = true; 
		   } else 
		   {
		      Identify_Person = false; 
		   }
		}

		if((!Initialize)&&(!Inview)) 
		{
		  Identify_start = true; 
		  Number_frame = 0; 
		}

		if (RGBimage.data)
		{
			if((Identify_Person)&&(Inview))
			{
              int fontFace = cv::FONT_HERSHEY_PLAIN ;
		      double fontScale = 5;
		      int thickness = 3;  
		      cv::putText(RGBimage, "Identify Person", cv::Point(220, 50), fontFace, fontScale,
			  CV_RGB(255, 0, 0), thickness, 8);
			}
		    cv::Size smallsize(640, 480); 
			cv::Mat mg1_small; 
			cv::resize(RGBimage, mg1_small, smallsize); 
            cv::imshow("Color & Skeleton", mg1_small);				
		}

        int key = cv::waitKey(10);
        if (key == 'q'){
            break;
        }
    }
 
    // de-initialize Kinect Sensor
    CHECKERROR(kinectSensor->Close());
    SAFERELEASE(kinectSensor);
    return 0;
}