
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
??
*/
// TestVideoSegmentation.cpp : Defines the entry point for the console application.
//

#include "libfreenect2grabber.h"


using namespace std;
using namespace cv;
y<
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

//DO I NEED THIS? YES START PTHREAD IN 
DWORD ProcessThread(LPVOID pParam) {
	pcl:: *p = (pcl::Libfreenect2Grabber*) pParam;
	p->ProcessThreadInternal();

	return 0;
}

template <typename T> inline T Clamp(T a, T minn, T maxx)
{ return (a < minn) ? minn : ( (a > maxx) ? maxx : a ); }

namespace pcl {
	Libfreenect2Grabber::Libfreenect2Grabber(const int instance) {


/*		HRESULT hr;
		int num = 0;
		m_person = m_depthStarted = m_videoStarted = m_audioStarted = m_infraredStarted = false;
		hStopEvent = NULL;
		hKinectThread = NULL;
		m_largeCloud = false;

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
*/

		undistorted_ = libfreenect2::Frame(512, 424, 4);
		/* Params Image & Depth */
		registered_ = libfreenect2::Frame(512, 424, 4);
		big_mat_ = libfreenect2::Frame(1920, 1082, 4);
		listener_ = libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

		signal(SIGINT,sigint_handler);

		if(freenect2_.enumerateDevices() == 0)
		{
			std::cout << "no kinect2 connected!" << std::endl;
			exit(-1);
		}

		serial_ = freenect2_.getDefaultDeviceSerialNumber();
		std::cout << "creating OpenCL processor" << std::endl;
		pipeline_ = new libfreenect2::OpenCLPacketPipeline();

		dev_ = freenect2_.openDevice(serial_, pipeline_);

/*
		if (!m_pKinectSensor || FAILED(hr)) {
			throw exception("No ready Kinect found");
		}*/
		m_colorSize = Size(cColorWidth, cColorHeight);
		m_depthSize = Size(cDepthWidth, cDepthHeight);
		m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
		m_pColorCoordinates = new ColorSpacePoint[cDepthHeight * cDepthWidth];
		m_pCameraSpacePoints = new CameraSpacePoint[cColorHeight * cColorWidth];

		// create callback signals
		image_signal_             = createSignal<sig_cb_libfreenect_image> ();
		depth_image_signal_    = createSignal<sig_cb_libfreenect_depth_image> ();
		image_depth_image_signal_    = createSignal<sig_cb_libfreenect_image_depth_image> ();
		point_cloud_rgba_signal_  = createSignal<sig_cb_libfreenect_point_cloud_rgba> ();
		all_data_signal_  = createSignal<sig_cb_libfreenect_all_data> ();
		/*ir_image_signal_       = createSignal<sig_cb_microsoft_ir_image> ();
		point_cloud_signal_    = createSignal<sig_cb_microsoft_point_cloud> ();
		point_cloud_i_signal_  = createSignal<sig_cb_microsoft_point_cloud_i> ();
		point_cloud_rgb_signal_   = createSignal<sig_cb_microsoft_point_cloud_rgb> ();
		*/

		rgb_sync_.addCallback (boost::bind (&Libfreenect2Grabber::imageDepthImageCallback, this, _1, _2));
	}

	void Libfreenect2Grabber::start() {
		block_signals();
		dev_->setColorFrameListener(&listener_);
		dev_->setIrAndDepthFrameListener(&listener_);
		dev_->start();

		registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());

		prepareMake3D(dev_->getIrCameraParams());
		//GetCameraSettings();
		/*hDepthMutex = CreateMutex(NULL,false,NULL);
		if(hDepthMutex == NULL)
		throw exception("Could not create depth mutex");
		hColorMutex = CreateMutex(NULL,false,NULL);
		if(hColorMutex == NULL)
		throw exception("Could not create color mutex");*/
		//hFrameEvent = (WAITABLE_HANDLE)CreateEvent(NULL,FALSE,FALSE,NULL);

		unblock_signals();
	}
	void Libfreenect2Grabber::prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p)
	{
		const int w = 512;
		const int h = 424;
			float * pm1 = colmap.data();
			float * pm2 = rowmap.data();
			for(int i = 0; i < w; i++)
			{
					*pm1++ = (i-depth_p.cx + 0.5) / depth_p.fx;
			}
			for (int i = 0; i < h; i++)
			{
					*pm2++ = (i-depth_p.cy + 0.5) / depth_p.fy;
			}
	}


	cv::Mat Libfreenect2Grabber::getColor(libfreenect2::Frame * colorFrameRef){
		libfreenect2::Frame * rgb = colorFrameRef;
		cv::Mat tmp(rgb->height, rgb->width, CV_8UC4, rgb->data);
		cv::Mat r = tmp.clone();
		return std::move(r);
	}

	cv::Mat Libfreenect2Grabber::getDepth(){
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];
		cv::Mat tmp(depth->height, depth->width, CV_8UC4, depth->data);
		cv::Mat r = tmp.clone();
		listener_.release(frames_);
		return std::move(r);
	}

	std::pair<cv::Mat, cv::Mat> Libfreenect2Grabber::getDepthRgb(){
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];
		libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
		registration_->apply(rgb, depth, &undistorted_, &registered_);
		cv::Mat tmp_depth(undistorted_.height, undistorted_.width, CV_8UC4, undistorted_.data);
		cv::Mat tmp_color(registered_.height, registered_.width, CV_8UC4, registered_.data);
		cv::Mat r = tmp_color.clone();
		cv::Mat d = tmp_depth.clone();
		listener_.release(frames_);
		return std::move(std::pair<cv::Mat, cv::Mat>(r,d));
	}


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Libfreenect2Grabber::getCloud(){

		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
		libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];

		registration_->apply(rgb, depth, &undistorted_, &registered_, true, &big_mat_);
		const short w = undistorted_.width;
		const short h = undistorted_.height;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(w, h));

		const float * itD0 = (float *)undistorted_.data;
		const char * itRGB0 = (char *)registered_.data;
		pcl::PointXYZRGB * itP = &cloud->points[0];

		for(int y = 0; y < h; ++y){

			const unsigned int offset = y * w;
			const float * itD = itD0 + offset;
			const char * itRGB = itRGB0 + offset * 4;
			const float dy = rowmap(y);

			for(size_t x = 0; x < w; ++x, ++itP, ++itD, itRGB += 4 )
			{
				const float depth_value = *itD / 1000.0f;

				if(!isnan(depth_value) && !(abs(depth_value) < 0.0001)){

					const float rx = colmap(x) * depth_value;
									const float ry = dy * depth_value;
					itP->z = depth_value;
					itP->x = rx;
					itP->y = ry;

					itP->b = itRGB[0];
					itP->g = itRGB[1];
					itP->r = itRGB[2];
				}
			}
		}
		listener_.release(frames_);
		return cloud;
	}
	void Libfreenect2Grabber::stop() {

		dev_->stop();
		dev_->close();
	/*
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

			m_pMultiSourceFrameReader->UnsubscribeMultiSourceFrameArrived(hFrameEvent);
			CloseHandle((HANDLE)hFrameEvent);
			hFrameEvent = NULL;
			/*CloseHandle(hDepthMutex);
			hDepthMutex = NULL;
			CloseHandle(hColorMutex);
			hColorMutex = NULL;
		}*/
		if (m_pColorRGBX) {
			delete [] m_pColorRGBX;
			m_pColorRGBX = NULL;
		}
		if(m_pColorCoordinates) {
			delete [] m_pColorCoordinates;
			m_pColorCoordinates = NULL;
		}
		if(m_pCameraSpacePoints) {
			delete [] m_pCameraSpacePoints;
			m_pCameraSpacePoints = NULL;
		}
	}

	bool Libfreenect2Grabber::isRunning () const {
		return (!(hKinectThread == NULL));
	}

	Libfreenect2Grabber::~Libfreenect2Grabber() {
		Release();
	}

	bool Libfreenect2Grabber::GetCameraSettings() {
		/*CameraSettings = NULL;
		if(S_OK == kinectInstance->NuiGetColorCameraSettings(&CameraSettings))
		CameraSettingsSupported = true;
		else
		CameraSettingsSupported = false;*/
		return CameraSettingsSupported;
	}

	void Libfreenect2Grabber::ProcessThreadInternal() {
	//	HANDLE handles[] = { reinterpret_cast<HANDLE>(hFrameEvent) };
		int idx;
		bool quit = false;
		while(!quit) {
			// Wait for any of the events to be signalled
			//idx = WaitForMultipleObjects(1,handles,FALSE,100);
			if (listener_.waitForNewFrame(frames_))
			{
					FrameArrived(frames_);
					listener_.release(frames_);
			}
			else
				continue;
/*
			switch(idx) {
			case WAIT_TIMEOUT:
				continue;
			case WAIT_OBJECT_0:
				IMultiSourceFrameArrivedEventArgs *pFrameArgs = nullptr;
				/*Hash id von Frame in EventArgs */
				/*Holen der Frame Reference in Frame Arrived
				HRESULT hr = m_pMultiSourceFrameReader->GetMultiSourceFrameArrivedEventData(hFrameEvent,&pFrameArgs);
				//frame arrived
				FrameArrived(pFrameArgs);
				pFrameArgs->Release();
				break;

				/*case WAIT_OBJECT_0 + 1:
				quit = true;
				continue;
			}*/
			//if(WaitForSingleObject(hStopEvent,1) == WAIT_OBJECT_0)
			//	quit = true;
			//else {
			//	//Get the newest frame info
			//	GetNextFrame();
			//}
		}
	}

	void Libfreenect2Grabber::Release() {
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

	string Libfreenect2Grabber::getName () const {
		return std::string ("Libfreenect2Grabber");
	}

	float Libfreenect2Grabber::getFramesPerSecond () const {
		return 30.0f;
	}

//Wofür?
/*
	void Libfreenect2Grabber::BodyIndexFrameArrived(IBodyIndexFrameReference* pBodyIndexFrameReference) {
		IBodyIndexFrame* pBodyIndexFrame = NULL;
		HRESULT hr = pBodyIndexFrameReference->AcquireFrame(&pBodyIndexFrame);
		if(FAILED(hr))
			return;
		//cout << "got a body index frame" << endl;
		IFrameDescription* pBodyIndexFrameDescription = NULL;
		int nBodyIndexWidth = 0;
		int nBodyIndexHeight = 0;
		UINT nBodyIndexBufferSize = 0;
		BYTE *pBodyIndexBuffer = NULL;

		// get body index frame data
		if (SUCCEEDED(hr)) {
			hr = pBodyIndexFrame->get_FrameDescription(&pBodyIndexFrameDescription);
		}
		if (SUCCEEDED(hr)) {
			hr = pBodyIndexFrameDescription->get_Width(&nBodyIndexWidth);
		}
		if (SUCCEEDED(hr)) {
			hr = pBodyIndexFrameDescription->get_Height(&nBodyIndexHeight);
		}
		if (SUCCEEDED(hr)) {
			hr = pBodyIndexFrame->AccessUnderlyingBuffer(&nBodyIndexBufferSize, &pBodyIndexBuffer);
		}
		SafeRelease(pBodyIndexFrameDescription);
		SafeRelease(pBodyIndexFrame);
	}*/

	void Libfreenect2Grabber::FrameArrived(libfreenect2::FrameMap *pArgs) {


					ColorFrameArrived(frames_[libfreenect2::Frame::Color])
					DepthFrameArrived(frames_[libfreenect2::Frame::Depth]);
	/*	HRESULT hr;
		IMultiSourceFrameReference *pFrameReference = nullptr;

		//cout << "got a valid frame" << endl;
		hr = pArgs->get_FrameReference(&pFrameReference);
		if (SUCCEEDED(hr))
		{
			IMultiSourceFrame *pFrame = nullptr;
			hr = pFrameReference->AcquireFrame(&pFrame);
			if (FAILED(hr)) {
				cout << "fail on AcquireFrame" << endl;
			}
			IColorFrameReference* pColorFrameReference = nullptr;
			IDepthFrameReference* pDepthFrameReference = nullptr;
			IBodyIndexFrameReference* pBodyIndexFrameReference = nullptr;
			hr = pFrame->get_DepthFrameReference(&pDepthFrameReference);
			if (SUCCEEDED(hr))
				DepthFrameArrived(pDepthFrameReference);
			SafeRelease(pDepthFrameReference);


			hr = pFrame->get_ColorFrameReference(&pColorFrameReference);
			if (SUCCEEDED(hr))
				ColorFrameArrived(pColorFrameReference);
			SafeRelease(pColorFrameReference);

			hr = pFrame->get_BodyIndexFrameReference(&pBodyIndexFrameReference);
			if (SUCCEEDED(hr))
				BodyIndexFrameArrived(pBodyIndexFrameReference);
			SafeRelease(pBodyIndexFrameReference);

			pFrameReference->Release();
		}
		*/
	}

#pragma endregion

	//Camera Functions
#pragma region Camera
	void Libfreenect2Grabber::ColorFrameArrived(libfreenect2::Frame * colorFrameRef) {

				//WaitForSingleObject(hColorMutex,INFINITE);
				//cout << "creating the image" << endl;
				Mat tmp = getColor(colorFrameRef);
				boost::shared_ptr<Mat> img(new Mat());
				*img = tmp.clone();
				if (image_signal_->num_slots () > 0) {
					//cout << "img signal num slot!" << endl;
					image_signal_->operator()(img);
				}
				if (num_slots<sig_cb_microsoft_point_cloud_rgba>() > 0 || all_data_signal_->num_slots() > 0 || image_depth_image_signal_->num_slots() > 0)
					rgb_sync_.add0 (img, m_rgbTime);
				//ReleaseMutex(hColorMutex);
			}


	}
#pragma endregion

	//Depth Functions
#pragma region Depth
	void Libfreenect2Grabber::DepthFrameArrived(IDepthFrameReference* pDepthFrameReference) {

			//WaitForSingleObject(hDepthMutex,INFINITE);
			Mat tmp = getDepth();

			MatDepth depth_img = *((MatDepth*)&(tmp.clone()));
			m_depthTime = nDepthTime;
			if (depth_image_signal_->num_slots () > 0) {
				depth_image_signal_->operator()(depth_img);
			}
			if (num_slots<sig_cb_microsoft_point_cloud_rgba>() > 0 || all_data_signal_->num_slots() > 0 || image_depth_image_signal_->num_slots() > 0)
				rgb_sync_.add1 (depth_img, m_depthTime);
			//ReleaseMutex(hDepthMutex);
		}

	}
#pragma endregion

#pragma region Cloud
	void Libfreenect2Grabber::imageDepthImageCallback (const boost::shared_ptr<Mat> &image,
		const MatDepth &depth_image)
	{
		boost::shared_ptr<PointCloud<PointXYZRGBA>> cloud;
		// check if we have color point cloud slots
		if(point_cloud_rgba_signal_->num_slots() > 0 || all_data_signal_->num_slots() > 0)
			cloud = convertToXYZRGBAPointCloud(image, depth_image);
		if (point_cloud_rgba_signal_->num_slots () > 0)
			point_cloud_rgba_signal_->operator()(cloud);
		if(all_data_signal_->num_slots() > 0) {
			//boost::shared_ptr<KinectData> data (new KinectData(image,depth_image,*cloud));
			//all_data_signal_->operator()(data);
		}

		if(image_depth_image_signal_->num_slots() > 0) {
			float constant = 1.0f;
			image_depth_image_signal_->operator()(image,depth_image,constant);
		}
	}

	void Libfreenect2Grabber::GetPointCloudFromData(const boost::shared_ptr<Mat> &img, const MatDepth &depth, boost::shared_ptr<PointCloud<PointXYZRGBA>> &cloud, bool alignToColor, bool preregistered) const
	{
		if(!img || img->empty() || depth.empty()) {
			cout << "empty img or depth" << endl;
			return;
		}
//Prepare3d?
		UINT16 *pDepth = (UINT16*)depth.data;
		int length = cDepthHeight * cDepthWidth, length2;
		HRESULT hr;
		if(alignToColor) {
			length2 = cColorHeight * cColorWidth;
			hr = m_pCoordinateMapper->MapColorFrameToCameraSpace(length,pDepth,length2,m_pCameraSpacePoints);
			if(FAILED(hr))
				throw exception("Couldn't map to camera!");
		} else {
			hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(length,pDepth,length,m_pCameraSpacePoints);
			if(FAILED(hr))
				throw exception("Couldn't map to camera!");
			hr = m_pCoordinateMapper->MapCameraPointsToColorSpace(length,m_pCameraSpacePoints,length,m_pColorCoordinates);
			if(FAILED(hr))
				throw exception("Couldn't map color!");
		}

		PointCloud<PointXYZRGBA>::iterator pCloud = cloud->begin();
		ColorSpacePoint *pColor = m_pColorCoordinates;
		CameraSpacePoint *pCamera = m_pCameraSpacePoints;
		float bad_point = std::numeric_limits<float>::quiet_NaN ();
		int x,y, safeWidth = cColorWidth - 1, safeHeight = cColorHeight - 1;
		int width = alignToColor ? cColorWidth : cDepthWidth;
		int height = alignToColor ? cColorHeight : cDepthHeight;
		for(int j = 0; j < height; j++) {
			for(int i = 0; i < width; i++) {
				PointXYZRGBA loc;
				Vec4b color;
				if(!preregistered && !alignToColor) {
					x = Clamp<int>(int(pColor->X),0,safeWidth);
					y = Clamp<int>(int(pColor->Y),0,safeHeight);
					//int index = y * cColorHeight + x;
					color = img->at<Vec4b>(y,x);
				} else
					color = img->at<Vec4b>(j,i);
				loc.b = color[0];
				loc.g = color[1];
				loc.r = color[2];
				loc.a = 255;
				if(pCamera->Z == 0) {
					loc.x = loc.y = loc.z = bad_point;
				} else {
					loc.x = pCamera->X;
					loc.y = pCamera->Y;
					loc.z = pCamera->Z;
				}
				//cout << "Iter: " << i << ", " << j << endl;
				*pCloud = loc;
				++pCamera; ++pCloud; ++pColor;
			}
		}
		img->release();
	}

	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> Libfreenect2Grabber::convertToXYZRGBAPointCloud (const boost::shared_ptr<cv::Mat> &image,
		const MatDepth &depth_image) const {

			boost::shared_ptr<PointCloud<PointXYZRGBA> > cloud (new PointCloud<PointXYZRGBA>);

			cloud->header.frame_id =  "/libfreenect_microsoft_rgb_optical_frame";
	/*		cloud->height = m_largeCloud ? cColorHeight : cDepthHeight;
			cloud->width = m_largeCloud ? cColorWidth : cDepthWidth;
			cloud->is_dense = false;
			cloud->points.resize (cloud->height * cloud->width);
			GetPointCloudFromData(image,depth_image,cloud,m_largeCloud,false);
			*/
			cloud = getCloud();
			cloud->sensor_origin_.setZero ();
			cloud->sensor_orientation_.w () = 1.0;
			cloud->sensor_orientation_.x () = 0.0;
			cloud->sensor_orientation_.y () = 0.0;
			cloud->sensor_orientation_.z () = 0.0;




			return cloud;
	}

#pragma endregion
};
