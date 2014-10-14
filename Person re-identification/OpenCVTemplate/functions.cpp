

#include <iostream>
#include <sstream>
#include "functions.h"

void processRGBData(cv::Mat& RGBimage, IColorFrameReader*colorFrameReader) { 
    IColorFrame *data = nullptr;
    IFrameDescription *frameDesc = nullptr;
    HRESULT hr = -1;
    RGBQUAD *colorBuffer = nullptr;
 
    hr = colorFrameReader->AcquireLatestFrame(&data); 
    if (SUCCEEDED(hr)) hr = data->get_FrameDescription(&frameDesc);
    if (SUCCEEDED(hr)) {
        int height = 0, width = 0;
        if (SUCCEEDED(frameDesc->get_Height(&height)) && 
            SUCCEEDED(frameDesc->get_Width(&width))) {
            colorBuffer = new RGBQUAD[height * width];
            hr = data->CopyConvertedFrameDataToArray(height * width * sizeof(RGBQUAD),
                 reinterpret_cast<BYTE*>(colorBuffer), ColorImageFormat_Bgra);
            if (SUCCEEDED(hr)) {
                cv:: Mat img1(height, width, CV_8UC4,
                    reinterpret_cast<void*>(colorBuffer));

				img1.copyTo(RGBimage); 			
            }
        }
    }
    if (colorBuffer != nullptr) {
        delete[] colorBuffer;
        colorBuffer = nullptr;
    }
    SAFERELEASE(data);
}

void processBodyData(cv::Mat img, IBodyFrameReader* bodyFrameReader1, ICoordinateMapper* m_pCoordinateMapper, bool& Inview)
{   
    IBodyFrame *pBodyFrame = NULL;
	HRESULT hr = -1;
    hr = bodyFrameReader1->AcquireLatestFrame(&pBodyFrame);

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;
        hr = pBodyFrame->get_RelativeTime(&nTime);
        IBody* ppBodies[BODY_COUNT] = {0};

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);		
        }

        if (SUCCEEDED(hr))
        {
            for (int i = 0; i < BODY_COUNT; ++i)
            {
                IBody* pBody = ppBodies[i];
                if (pBody)
                {
                    BOOLEAN bTracked = false;
                    hr = pBody->get_IsTracked(&bTracked);

                    if (SUCCEEDED(hr) && bTracked)
                    {
                        Joint joints[JointType_Count];						
						ColorSpacePoint colorPoint = { 0 };
						cv::Point point2D[JointType_Count]; 

                        hr = pBody->GetJoints(_countof(joints), joints);  
                        if (SUCCEEDED(hr))
                        {			
							int Number_joints = 0; 
                            for (int j = 0; j < _countof(joints); ++j)
                            {                               
                               m_pCoordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &colorPoint);
							 
							   int x = static_cast<int>( colorPoint.X );
                               int y = static_cast<int>( colorPoint.Y );

							   point2D[j].x = x; 
							   point2D[j].y = y; 

							   if( ( x >= 0 ) && ( x < img.cols) && ( y >= 0 ) && ( y < img.rows ) )
							   {
								 Number_joints++; 
                                 cv::circle( img, cv::Point( x, y ), 10,  cv::Scalar( 128, 128, 0 ), -1, CV_AA );
							   }
							} 
							// draw skeleton body 
							// spine 
							if (Number_joints == 25)   
							{
								Inview = true; 
							}else 
							{
								Inview = false; 
							}
							cv::line(img, point2D[0], point2D[1],cv::Scalar( 128, 128, 0 ), 5, 8, 0);
							cv::line(img, point2D[1], point2D[2],cv::Scalar( 128, 128, 0 ), 5, 8, 0);
							cv::line(img, point2D[2], point2D[3],cv::Scalar( 128, 128, 0 ), 5, 8, 0);
							// left arm 
							cv::line(img, point2D[2], point2D[4],cv::Scalar( 128, 128, 0 ), 5, 8, 0);
							cv::line(img, point2D[4], point2D[5],cv::Scalar( 128, 128, 0 ), 5, 8, 0);
							cv::line(img, point2D[5], point2D[6],cv::Scalar( 128, 128, 0 ), 5, 8, 0);
							cv::line(img, point2D[6], point2D[7],cv::Scalar( 128, 128, 0 ), 5, 8, 0);
							// right arm 
							cv::line(img, point2D[2], point2D[8],cv::Scalar( 128, 128, 0 ), 5, 8, 0);
							cv::line(img, point2D[8], point2D[9],cv::Scalar( 128, 128, 0 ), 5, 8, 0);
							cv::line(img, point2D[9], point2D[10],cv::Scalar( 128, 128, 0 ), 5, 8, 0);
							cv::line(img, point2D[10], point2D[11],cv::Scalar( 128, 128, 0 ), 5, 8, 0);
							// left leg 
							cv::line(img, point2D[12], point2D[13],cv::Scalar( 128, 128, 0 ), 5, 8, 0);
							cv::line(img, point2D[13], point2D[14],cv::Scalar( 128, 128, 0 ), 5, 8, 0);
							cv::line(img, point2D[14], point2D[15],cv::Scalar( 128, 128, 0 ), 5, 8, 0);
							// right leg 
							cv::line(img, point2D[16], point2D[17],cv::Scalar( 128, 128, 0 ), 5, 8, 0);
							cv::line(img, point2D[17], point2D[18],cv::Scalar( 128, 128, 0 ), 5, 8, 0);
							cv::line(img, point2D[18], point2D[19],cv::Scalar( 128, 128, 0 ), 5, 8, 0);
                        }
                    }
                 }
              }      
          }
	} 
    SAFERELEASE(pBodyFrame);
}

void initializeBodyFeature(double& Feature_Head_height, double& Feature_Torso_ratio, IBodyFrameReader*  bodyFrameReader2)
{   
    int totalframes = 0;  
	int Number_of_frames_for_initialization = 5;
	double Sum_Head_height = 0; 
	double Sum_Torso_ratio = 0; 

    while (1)
	{	
			if (totalframes == Number_of_frames_for_initialization)  
			{
				Feature_Head_height = Sum_Head_height/Number_of_frames_for_initialization; 
				Feature_Torso_ratio = Sum_Torso_ratio/Number_of_frames_for_initialization; 
				std::cout << "average" << Feature_Head_height << " " << Feature_Torso_ratio << std::endl;
				break; 
			}

			IBodyFrame *pBodyFrame = NULL;
			HRESULT hr = -1;
			hr = bodyFrameReader2->AcquireLatestFrame(&pBodyFrame);

			if (SUCCEEDED(hr))
			{     		
				IBody* ppBodies[BODY_COUNT] = {0};
				Vector4 floorplane_vector; 

				INT64 nTime = 0;
				hr = pBodyFrame->get_RelativeTime(&nTime);

				if (SUCCEEDED(hr))
				{
					hr = pBodyFrame->get_FloorClipPlane(&floorplane_vector); 
					hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);		
				}

				float	FA = floorplane_vector.x; 
				float   FB = floorplane_vector.y; 
				float	FC = floorplane_vector.z; 	
				float	FD = floorplane_vector.w; 
				float	F_norm = sqrt(FA*FA + FB*FB + FC*FC);	

				if (SUCCEEDED(hr))
				{
					for (int i = 0; i < BODY_COUNT; ++i)
					{
						IBody* pBody = ppBodies[i];
						if (pBody)
						{
							BOOLEAN bTracked = false;
							hr = pBody->get_IsTracked(&bTracked);

							if (SUCCEEDED(hr) && bTracked)
							{
								totalframes++; 
								Joint joints[JointType_Count];						
								ColorSpacePoint colorPoint = { 0 };
								cv::Point point2D[JointType_Count]; 
			
								hr = pBody->GetJoints(_countof(joints), joints);  					

								CameraSpacePoint head3D, neck, shoulder_left, shoulder_right, hip_left, hip_right; 
								double headtofloor, torso_ratio; 

								if (SUCCEEDED(hr))
								{						    
								   // calculate head height   	
								   head3D = joints[3].Position; 
								   headtofloor = (FA*head3D.X + FB*head3D.Y + FC*head3D.Z + FD)/F_norm; 
								   Sum_Head_height += headtofloor; 

								   // calculate torso-ratio
								   neck = joints[2].Position; 
								   shoulder_left   = joints[4].Position; 
								   shoulder_right  = joints[8].Position; 
								   hip_left        = joints[12].Position; 
								   hip_right       = joints[16].Position; 

								   double dis_shoulder_center_left = sqrt (pow((neck.X - shoulder_left.X), 2) +   
																		   pow((neck.Y - shoulder_left.Y), 2) +
																		   pow((neck.Z - shoulder_left.Z), 2)); 
								   double dis_shoulder_center_right = sqrt (pow((neck.X - shoulder_right.X), 2) +   
																		   pow((neck.Y - shoulder_right.Y), 2) +
																		   pow((neck.Z - shoulder_right.Z), 2)); 
							                                
								  double hip_left_floor  = (FA*hip_left.X + FB*hip_left.Y + FC*hip_left.Z + FD)/F_norm; 
								  double hip_right_floor = (FA*hip_right.X + FB*hip_right.Y + FC*hip_right.Z + FD)/F_norm; 
								  torso_ratio = (dis_shoulder_center_left + dis_shoulder_center_right)/
																 (hip_left_floor + hip_right_floor);
								  Sum_Torso_ratio += torso_ratio; 

								  std::cout << totalframes<< " " << headtofloor << " " << torso_ratio << std::endl; 
								  break; 
								}								
							}
						 }
					  }      
				  }
			} 
			SAFERELEASE(pBodyFrame);		
		    int key = cv::waitKey(10);
	}
}