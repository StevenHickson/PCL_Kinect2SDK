
/*
Copyright (C) 2014 Steven Hickson

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA

*/
// TestVideoSegmentation.cpp : Defines the entry point for the console application.
//

#include "Microsoft_grabber2.h"

using namespace std;
using namespace cv;

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

DWORD ProcessThread(LPVOID pParam) {
	pcl::Microsoft2Grabber *p = (pcl::Microsoft2Grabber*) pParam;
	p->ProcessThreadInternal();
	return 0;
}

template <typename T> inline T Clamp(T a, T minn, T maxx)
{ return (a < minn) ? minn : ( (a > maxx) ? maxx : a ); }

namespace pcl {
	Microsoft2Grabber::Microsoft2Grabber(const int instance) {
		HRESULT hr;
		int num = 0;
		m_person = m_depthStarted = m_videoStarted = m_audioStarted = m_infraredStarted = false;
		hStopEvent = NULL;
		hKinectThread = NULL;

		hr = GetDefaultKinectSensor(&m_pKinectSensor);
		if (FAILED(hr)) {
			throw exception("Error could not get default kinect sensor");
		}

		if (m_pKinectSensor) {
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
			hr = m_pKinectSensor->Open();
			if (SUCCEEDED(hr)) {
				hr = m_pKinectSensor->OpenMultiSourceFrameReader(
					FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_BodyIndex,
					&m_pMultiSourceFrameReader);
				if (SUCCEEDED(hr))
				{
					m_videoStarted = m_depthStarted = true;
				} else
					throw exception("Failed to Open Kinect Multisource Stream");
			}
		}

		if (!m_pKinectSensor || FAILED(hr)) {
			throw exception("No ready Kinect found");
		}
		m_colorSize = Size(cColorWidth, cColorHeight);
		m_depthSize = Size(cDepthWidth, cDepthHeight);
		m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];

		// create callback signals
		image_signal_             = createSignal<sig_cb_microsoft_image> ();
		depth_image_signal_    = createSignal<sig_cb_microsoft_depth_image> ();
		point_cloud_rgba_signal_  = createSignal<sig_cb_microsoft_point_cloud_rgba> ();
		all_data_signal_  = createSignal<sig_cb_microsoft_all_data> ();
		/*ir_image_signal_       = createSignal<sig_cb_microsoft_ir_image> ();
		point_cloud_signal_    = createSignal<sig_cb_microsoft_point_cloud> ();
		point_cloud_i_signal_  = createSignal<sig_cb_microsoft_point_cloud_i> ();
		point_cloud_rgb_signal_   = createSignal<sig_cb_microsoft_point_cloud_rgb> ();
		*/ 
		rgb_sync_.addCallback (boost::bind (&Microsoft2Grabber::imageDepthImageCallback, this, _1, _2));
	}

	void Microsoft2Grabber::start() {
		block_signals();
		//GetCameraSettings();
		/*hDepthMutex = CreateMutex(NULL,false,NULL);
		if(hDepthMutex == NULL)
			throw exception("Could not create depth mutex");
		hColorMutex = CreateMutex(NULL,false,NULL);
		if(hColorMutex == NULL)
			throw exception("Could not create color mutex");*/
		hStopEvent = CreateEvent( NULL, FALSE, FALSE, NULL );
		hKinectThread = CreateThread( NULL, 0, (LPTHREAD_START_ROUTINE)&ProcessThread, this, 0, NULL );
		//boost::this_thread::sleep (boost::posix_time::seconds (1));
		unblock_signals();
	}

	void Microsoft2Grabber::stop() {
		//stop the ProcessThread
		if(hStopEvent != NULL) {
			//signal the process to stop
			SetEvent(hStopEvent);
			if(hKinectThread != NULL) {
				WaitForSingleObject(hKinectThread,INFINITE);
				CloseHandle(hKinectThread);
				hKinectThread = NULL;
			}
			CloseHandle(hStopEvent);
			hStopEvent = NULL;
			/*CloseHandle(hDepthMutex);
			hDepthMutex = NULL;
			CloseHandle(hColorMutex);
			hColorMutex = NULL;*/
		}
		if (m_pColorRGBX) {
			delete [] m_pColorRGBX;
			m_pColorRGBX = NULL;
		}
	}

	bool Microsoft2Grabber::isRunning () const {
		return (!(hKinectThread == NULL));
	}

	Microsoft2Grabber::~Microsoft2Grabber() {
		Release();
	}

	bool Microsoft2Grabber::GetCameraSettings() {
		/*CameraSettings = NULL;
		if(S_OK == kinectInstance->NuiGetColorCameraSettings(&CameraSettings))
		CameraSettingsSupported = true;
		else
		CameraSettingsSupported = false;*/
		return CameraSettingsSupported;
	}

	void Microsoft2Grabber::ProcessThreadInternal() {
		bool quit = false;
		while(!quit) {
			// Wait for any of the events to be signalled
			if(WaitForSingleObject(hStopEvent,1) == WAIT_OBJECT_0)
				quit = true;
			else {
				//Get the newest frame info
				GetNextFrame();
			}
		}
	}

	void Microsoft2Grabber::Release() {
		try {
			//clean up stuff here
			stop();
			if(m_pKinectSensor) {
				//Shutdown NUI and Close handles
				if (m_pMultiSourceFrameReader)
					SafeRelease(m_pMultiSourceFrameReader);
				if(m_pCoordinateMapper)
					SafeRelease(m_pCoordinateMapper);
				// close the Kinect Sensor
				if (m_pKinectSensor)
					m_pKinectSensor->Close();

				SafeRelease(m_pKinectSensor);
			}
		} catch(...) {
			//destructor never throws
		}
	}

	string Microsoft2Grabber::getName () const {
		return std::string ("Microsoft2Grabber");
	}

	float Microsoft2Grabber::getFramesPerSecond () const {
		return 30.0f;
	}

	void Microsoft2Grabber::GetNextFrame() {
		if (!m_pMultiSourceFrameReader)
		{
			return;
		}

		IMultiSourceFrame* pMultiSourceFrame = NULL;
		IDepthFrame* pDepthFrame = NULL;
		IColorFrame* pColorFrame = NULL;
		IBodyIndexFrame* pBodyIndexFrame = NULL;

		HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

		if (SUCCEEDED(hr))
		{
			IDepthFrameReference* pDepthFrameReference = NULL;

			hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
			if (SUCCEEDED(hr))
			{
				hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
			}

			SafeRelease(pDepthFrameReference);
		}

		if (SUCCEEDED(hr))
		{
			IColorFrameReference* pColorFrameReference = NULL;

			hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
			if (SUCCEEDED(hr))
			{
				hr = pColorFrameReference->AcquireFrame(&pColorFrame);
			}

			SafeRelease(pColorFrameReference);
		}

		if (SUCCEEDED(hr))
		{
			IBodyIndexFrameReference* pBodyIndexFrameReference = NULL;

			hr = pMultiSourceFrame->get_BodyIndexFrameReference(&pBodyIndexFrameReference);
			if (SUCCEEDED(hr))
			{
				hr = pBodyIndexFrameReference->AcquireFrame(&pBodyIndexFrame);
			}

			SafeRelease(pBodyIndexFrameReference);
		}

		if (SUCCEEDED(hr))
		{
			INT64 nDepthTime = 0;
			IFrameDescription* pDepthFrameDescription = NULL;
			int nDepthWidth = 0;
			int nDepthHeight = 0;
			UINT nDepthBufferSize = 0;

			IFrameDescription* pColorFrameDescription = NULL;
			int nColorWidth = 0;
			int nColorHeight = 0;
			ColorImageFormat imageFormat = ColorImageFormat_None;
			UINT nColorBufferSize = 0;
			RGBQUAD *pColorBuffer = NULL;

			IFrameDescription* pBodyIndexFrameDescription = NULL;
			int nBodyIndexWidth = 0;
			int nBodyIndexHeight = 0;
			UINT nBodyIndexBufferSize = 0;
			BYTE *pBodyIndexBuffer = NULL;

			// get depth frame data

			hr = pDepthFrame->get_RelativeTime(&nDepthTime);

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
			}

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrameDescription->get_Width(&nDepthWidth);
			}

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrameDescription->get_Height(&nDepthHeight);
			}

			if (SUCCEEDED(hr))
			{
				//m_pDepthBuffer = new UINT16[cDepthWidth * cDepthHeight];
				hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &m_pDepthBuffer);
				//pDepthFrame->CopyFrameDataToArray(nDepthBufferSize,m_pDepthBuffer);
				//WaitForSingleObject(hDepthMutex,INFINITE);
				Mat tmp = Mat(m_depthSize, DEPTH_PIXEL_TYPE, m_pDepthBuffer, Mat::AUTO_STEP);
				boost::shared_ptr<MatDepth> depth_img((MatDepth*)(new Mat(cDepthHeight,cDepthWidth,DEPTH_PIXEL_TYPE)));
				//total hack for now
				depth_img->operator=(*((MatDepth*)&(tmp.clone()))); //need to deep copy because of the call to SafeRelease(pDepthFrame) to prevent access violation
				m_depthTime = nDepthTime;
				if (depth_image_signal_->num_slots () > 0) {
					//cout << "img signal num slot!" << endl;
					depth_image_signal_->operator()(depth_img);
				}
				if (num_slots<sig_cb_microsoft_point_cloud_rgba>() > 0 || all_data_signal_->num_slots() > 0)
					rgb_sync_.add1 (depth_img, m_depthTime);
				//ReleaseMutex(hDepthMutex);
			}

			// get color frame data

			if (SUCCEEDED(hr))
			{
				hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
			}

			if (SUCCEEDED(hr))
			{
				hr = pColorFrameDescription->get_Width(&nColorWidth);
			}

			if (SUCCEEDED(hr))
			{
				hr = pColorFrameDescription->get_Height(&nColorHeight);
			}

			if (SUCCEEDED(hr))
			{
				hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
			}

			if (SUCCEEDED(hr))
			{
				if (imageFormat == ColorImageFormat_Bgra)
				{
					hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
				}
				else if (m_pColorRGBX)
				{
					pColorBuffer = m_pColorRGBX;
					nColorBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
					hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
				}
				else
				{
					hr = E_FAIL;
				}
				if(SUCCEEDED(hr)) {
					//WaitForSingleObject(hColorMutex,INFINITE);
					boost::shared_ptr<Mat> img(new Mat(cColorHeight,cColorWidth,COLOR_PIXEL_TYPE));
					*img = Mat(m_colorSize, COLOR_PIXEL_TYPE, pColorBuffer, Mat::AUTO_STEP);
					m_rgbTime = nDepthTime;
					if (image_signal_->num_slots () > 0) {
						//cout << "img signal num slot!" << endl;
						image_signal_->operator()(img);
					}
					if (num_slots<sig_cb_microsoft_point_cloud_rgba>() > 0 || all_data_signal_->num_slots() > 0)
						rgb_sync_.add0 (img, m_rgbTime);
					//ReleaseMutex(hColorMutex);
				}
			}

			// get body index frame data

			if (SUCCEEDED(hr))
			{
				hr = pBodyIndexFrame->get_FrameDescription(&pBodyIndexFrameDescription);
			}

			if (SUCCEEDED(hr))
			{
				hr = pBodyIndexFrameDescription->get_Width(&nBodyIndexWidth);
			}

			if (SUCCEEDED(hr))
			{
				hr = pBodyIndexFrameDescription->get_Height(&nBodyIndexHeight);
			}

			if (SUCCEEDED(hr))
			{
				hr = pBodyIndexFrame->AccessUnderlyingBuffer(&nBodyIndexBufferSize, &pBodyIndexBuffer);            
			}

			SafeRelease(pDepthFrameDescription);
			SafeRelease(pColorFrameDescription);
			SafeRelease(pBodyIndexFrameDescription);
		}

		SafeRelease(pDepthFrame);
		SafeRelease(pColorFrame);
		SafeRelease(pBodyIndexFrame);
		SafeRelease(pMultiSourceFrame);
	}

#pragma endregion

	//Camera Functions
#pragma region Camera

#pragma endregion

	//Depth Functions
#pragma region Depth

#pragma endregion

#pragma region Cloud
	void Microsoft2Grabber::imageDepthImageCallback (const boost::shared_ptr<Mat> &image,
		const boost::shared_ptr<MatDepth> &depth_image)
	{
		boost::shared_ptr<PointCloud<PointXYZRGBA>> cloud;
		// check if we have color point cloud slots
		if(point_cloud_rgba_signal_->num_slots() > 0 || all_data_signal_->num_slots() > 0)
			cloud = convertToXYZRGBAPointCloud(image, depth_image);
		if (point_cloud_rgba_signal_->num_slots () > 0)
			point_cloud_rgba_signal_->operator()(cloud);
		if(all_data_signal_->num_slots() > 0) {
			boost::shared_ptr<KinectData> data (new KinectData(*image,*depth_image,*cloud));
			all_data_signal_->operator()(data);
		}
	}

	void Microsoft2Grabber::GetPointCloudFromData(const boost::shared_ptr<Mat> &img, const boost::shared_ptr<MatDepth> &depth, boost::shared_ptr<PointCloud<PointXYZRGBA>> &cloud, bool useZeros, bool alignToColor, bool preregistered) const
	{
		/*
		assert(!img->empty() && !depth->empty());

		PointCloud<PointXYZRGBA>::iterator pCloud = cloud->begin();
		Mat_<int>::const_iterator pDepth = depth->begin<int>();
		int safeWidth = img->cols - 1, safeHeight = img->rows - 1, safeDepthWidth = depthWidth - 1, safeDepthHeight = depthHeight - 1;
		float cx_d = KINECT_CX_D, cy_d = KINECT_CY_D;
		bool res_320 = (depthWidth == 320);
		float bad_point = std::numeric_limits<float>::quiet_NaN ();
		for(int j = 0; j < depthHeight; j++) {
		for(int i = 0; i < depthWidth; i++) {
		PointXYZRGBA loc;
		Vec3b color;
		LONG x = res_320 ? i: i >> 1, y = res_320 ? j : j >> 1;
		if(!preregistered) {
		kinectInstance->NuiImageGetColorPixelCoordinatesFromDepthPixel(colorRes,NULL,LONG(320-x),LONG(y),(*pDepth)<<3,&x,&y);
		x = Clamp<int>(safeWidth-x,0,safeWidth);
		y = Clamp<int>(y,0,safeHeight);
		color = img->at<Vec3b>(y,x);
		} else
		color = img->at<Vec3b>(j,i);
		loc.b = color[0];
		loc.g = color[1];
		loc.r = color[2];
		loc.a = 255;
		if(*pDepth == 0) {
		loc.x = loc.y = loc.z = bad_point;
		} else {
		const double newDepth = (*pDepth * 0.001f); //convert from millimeters to meters
		loc.x = float(((float)i - cx_d) * newDepth * KINECT_FX_D);
		if(preregistered)
		loc.y = float(((float)j - cy_d) * newDepth * KINECT_FY_D);
		else
		loc.y = float(((float)(safeDepthHeight - j) - cy_d) * newDepth * KINECT_FY_D);
		loc.z = float(newDepth);
		}
		//cout << "Iter: " << i << ", " << j << endl;
		if(!preregistered && alignToColor) {
		(*cloud)(x,y) = loc;
		//for weird resolution differences
		if(imgWidth == depthWidth << 1) {
		//TOBE DONE LATER
		}
		} else {
		*pCloud = loc;
		}
		++pDepth; ++pCloud;
		}
		}
		if(!preregistered && alignToColor) {
		pCloud = cloud->begin();
		Mat_<Vec3b>::const_iterator pColor = img->begin<Vec3b>();
		while(pCloud != cloud->end()) {
		if(pCloud->z == 0) {
		pCloud->b = (*pColor)[0];
		pCloud->g = (*pColor)[1];
		pCloud->r = (*pColor)[2];
		pCloud->a = 255;
		}
		++pCloud; ++pColor;
		}
		}		*/
	}

	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> Microsoft2Grabber::convertToXYZRGBAPointCloud (const boost::shared_ptr<cv::Mat> &image,
		const boost::shared_ptr<MatDepth> &depth_image) const {
			boost::shared_ptr<PointCloud<PointXYZRGBA> > cloud (new PointCloud<PointXYZRGBA>);

			cloud->header.frame_id =  "/microsoft_rgb_optical_frame";
			cloud->height = std::max (cColorHeight, cDepthHeight);
			cloud->width = std::max (cColorWidth, cDepthWidth);
			cloud->is_dense = false;
			cloud->points.resize (cloud->height * cloud->width);
			GetPointCloudFromData(image,depth_image,cloud,true,false,false);
			cloud->sensor_origin_.setZero ();
			cloud->sensor_orientation_.w () = 1.0;
			cloud->sensor_orientation_.x () = 0.0;
			cloud->sensor_orientation_.y () = 0.0;
			cloud->sensor_orientation_.z () = 0.0;
			return (cloud);
	}

#pragma endregion
};