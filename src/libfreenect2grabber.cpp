
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

Modified for 
*/
// TestVideoSegmentation.cpp : Defines the entry point for the console application.
//

#include "libfreenect2grabber.h"


using namespace std;
using namespace cv;

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)

	{
        delete[] pInterfaceToRelease;
		pInterfaceToRelease = NULL;
	}
}

//DO I NEED THIS? YES START PTHREAD IN LINUX
static void* ProcessThread(void* pParam) {
	pcl::Libfreenect2Grabber *p = (pcl::Libfreenect2Grabber*) pParam;
	p->ProcessThreadInternal();
}

template <typename T> inline T Clamp(T a, T minn, T maxx)
{ return (a < minn) ? minn : ( (a > maxx) ? maxx : a ); }




namespace pcl {
	Libfreenect2Grabber::Libfreenect2Grabber() : activImageFrame(1920, 1082, 4),activDepthFrame(512, 424, 4), undistorted_(512, 424, 4), registered_(512, 424, 4), big_mat_(1920, 1082, 4), listener_(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth){


		if(freenect2_.enumerateDevices() == 0)
		{
			std::cout << "no kinect2 connected!" << std::endl;
			exit(-1);
		}

		serial_ = freenect2_.getDefaultDeviceSerialNumber();
		std::cout << "creating OpenCL processor" << std::endl;
        	pipeline_ = new libfreenect2::OpenCLPacketPipeline();

		dev_ = freenect2_.openDevice(serial_, pipeline_);

        	dev_->setColorFrameListener(&listener_);
        	dev_->setIrAndDepthFrameListener(&listener_);


		// create callback signals
		image_signal_             = createSignal<sig_cb_libfreenect_image> ();
		depth_image_signal_    = createSignal<sig_cb_libfreenect_depth_image> ();
		image_depth_image_signal_    = createSignal<sig_cb_libfreenect_image_depth_image> ();
		point_cloud_rgba_signal_  = createSignal<sig_cb_libfreenect_point_cloud_rgba> ();

        //All dataCallback?
		rgb_sync_.addCallback (boost::bind (&Libfreenect2Grabber::imageDepthImageCallback, this, _1, _2));
	}

	void Libfreenect2Grabber::start() {
		block_signals();
		int iret1 = 0;


		dev_->start();
        registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());
        prepareMake3D(dev_->getIrCameraParams());
        iret1 = pthread_create( &pKinectThread, NULL, ProcessThread,  this);
        if(iret1)
        {
					fprintf(stderr,"Error - pthread_create() return code: %d\n",iret1);
					exit(EXIT_FAILURE);
        }
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
		libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
		cv::Mat tmp(rgb->height, rgb->width, CV_8UC4, rgb->data);
		cv::Mat r = tmp.clone();
		return (r);
	}

	cv::Mat Libfreenect2Grabber::getDepth(libfreenect2::Frame * depthFrameRef){
		libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];
		cv::Mat tmp(depth->height, depth->width, CV_8UC4, depth->data);
		cv::Mat r = tmp.clone();
		return (r);
	}

	std::pair<cv::Mat, cv::Mat> Libfreenect2Grabber::getDepthRgb(libfreenect2::Frame * colorFrameRef, libfreenect2::Frame *depthFrameRef){
		libfreenect2::Frame * depth = depthFrameRef;
		libfreenect2::Frame * rgb = colorFrameRef;
		registration_->apply(rgb, depth, &undistorted_, &registered_);
		cv::Mat tmp_depth(undistorted_.height, undistorted_.width, CV_8UC4, undistorted_.data);
		cv::Mat tmp_color(registered_.height, registered_.width, CV_8UC4, registered_.data);
		cv::Mat r = tmp_color.clone();
		cv::Mat d = tmp_depth.clone();
		return std::move(std::pair<cv::Mat, cv::Mat>(r,d));
	}



 void Libfreenect2Grabber::getCloud(const boost::shared_ptr<cv::Mat> &img, const MatDepth &depth, boost::shared_ptr<PointCloud<PointXYZRGB>> &cloud)
 const {

	//	listener_.waitForNewFrame(frames_);
        libfreenect2::Frame rgb = libfreenect2::Frame(1920, 1082, 4);
        libfreenect2::Frame depthf = libfreenect2::Frame(512, 424, 4);

        rgb.data = (unsigned char*) img->data;
        depthf.data = (unsigned char*) depth.data;

        const libfreenect2::Frame *  rgb_ptr = const_cast<const libfreenect2::Frame *> (&rgb);
        const libfreenect2::Frame *  depthf_ptr =const_cast<const libfreenect2::Frame *> (&depthf);

        registration_->apply(rgb_ptr, depthf_ptr, &undistorted_, &registered_, true, &big_mat_);

        const short w = undistorted_.width;
        const short h = undistorted_.height;

      //cloud( = new )pcl::PointCloud<pcl::PointXYZRGB>(w, h);
        cloud->width = w;
        cloud->height = h;

  	cv::Mat tmp_itD0(undistorted_.height, undistorted_.width, CV_8UC4, undistorted_.data);
        cv::Mat tmp_itRGB0(registered_.height, registered_.width, CV_8UC4, registered_.data);
  
        cv::flip(tmp_itD0,tmp_itD0,1);
        cv::flip(tmp_itRGB0,tmp_itRGB0,1);

        const float * itD0 = (float *) tmp_itD0.ptr();
        const char * itRGB0 = (char *) tmp_itRGB0.ptr();
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
      //      listener_.release(frames_);

	}
	void Libfreenect2Grabber::stop() {

		dev_->stop();
        dev_->close();
        //pth
        pthread_cancel(pKinectThread);

		if (registration_) {
			delete [] registration_;
			registration_ = NULL;
		}

	}

	bool Libfreenect2Grabber::isRunning () const {
        return (!(pKinectThread == NULL));
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
			FrameArrived();
		}
	}

	void Libfreenect2Grabber::Release() {
		try {
			//clean up stuff here
			stop();
			if(dev_) {
				//Shutdown NUI and Close handles
				if (pipeline_)
					SafeRelease(pipeline_);
				// close the Kinect Sensor
				if (dev_)
					dev_->close();

				SafeRelease(dev_);
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
    ///TODO: BodyFrame?
    ///

	void Libfreenect2Grabber::FrameArrived() {

        listener_.waitForNewFrame(frames_);
					libfreenect2::Frame * color = frames_[libfreenect2::Frame::Color];
					libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];
                    if (color != NULL){
                       // printf("Test color %s \n",color->data);
						ColorFrameArrived(color);
                    }

                    if (depth != NULL){
                      //  printf("Test depth %s \n",depth->data);
						DepthFrameArrived(depth);
                    }
         listener_.release(frames_);
	}

	//Camera Functions
	void Libfreenect2Grabber::ColorFrameArrived(libfreenect2::Frame * colorFrameRef) {

				float timestamp = colorFrameRef->timestamp;
				//WaitForSingleObject(hColorMutex,INFINITE);
				//cout << "creating the image" << endl;
				Mat tmp = getColor(colorFrameRef);
			//	libfreenect2::Frame activImageFrame = *colorFrameRef;
				boost::shared_ptr<Mat> img(new Mat());
				*img = tmp.clone();
				if (image_signal_->num_slots () > 0) {
					//cout << "img signal num slot!" << endl;
					image_signal_->operator()(img);
				}
				if (num_slots<sig_cb_libfreenect_point_cloud_rgba>() > 0 || /*all_data_signal_->num_slots() > 0 ||*/ image_depth_image_signal_->num_slots() > 0)
					rgb_sync_.add0 (img, timestamp);
				//ReleaseMutex(hColorMutex);
			}




	//Depth Functions

	void Libfreenect2Grabber::DepthFrameArrived (libfreenect2::Frame * depthFrameRef) {

			float timestamp = depthFrameRef->timestamp;
			//WaitForSingleObject(hDepthMutex,INFINITE);
				Mat tmp = getDepth(depthFrameRef);
	//		libfreenect2::Frame activDepthFrame = *depthFrameRef;
                MatDepth depth_img = MatDepth()	;
                depth_img = *((MatDepth*)(&tmp.clone()));


	//		m_depthTime = nDepthTime;
			if (depth_image_signal_->num_slots () > 0) {
				depth_image_signal_->operator()(depth_img);
			}
			if (num_slots<sig_cb_libfreenect_point_cloud_rgba>() > 0 || /*all_data_signal_->num_slots() > 0 ||*/ image_depth_image_signal_->num_slots() > 0)
				rgb_sync_.add1 (depth_img, timestamp);
			//ReleaseMutex(hDepthMutex);
		}



	void Libfreenect2Grabber::imageDepthImageCallback (const boost::shared_ptr<Mat> &image, const MatDepth &depth_image)
	{

			boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
		// check if we have color point cloud slots
		if(point_cloud_rgba_signal_->num_slots() > 0 /*|| all_data_signal_->num_slots() > 0*/)
			cloud = convertToXYZRGBAPointCloud(image, depth_image);
		if (point_cloud_rgba_signal_->num_slots () > 0)
			point_cloud_rgba_signal_->operator()(cloud);
	//	if(all_data_signal_->num_slots() > 0) {
			//boost::shared_ptr<KinectData> data (new KinectData(image,depth_image,*cloud));
			//all_data_signal_->operator()(data);
	//	}

		if(image_depth_image_signal_->num_slots() > 0) {
			float constant = 1.0f;
			image_depth_image_signal_->operator()(image,depth_image,constant);
		}

        //listener_.release(frames_);
	}

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > Libfreenect2Grabber::convertToXYZRGBAPointCloud (const boost::shared_ptr<cv::Mat> &image,
		const MatDepth &depth_image) const {


        libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
        libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];

        registration_->apply(rgb, depth, &undistorted_, &registered_, true, &big_mat_);
        const short w = undistorted_.width;
        const short h = undistorted_.height;

     //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr = cloud.get();
        boost::shared_ptr<PointCloud<PointXYZRGB> > cloud (new PointCloud<PointXYZRGB>(w,h));
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
            cloud->sensor_origin_.setZero ();
            cloud->sensor_orientation_.w () = 1.0;
            cloud->sensor_orientation_.x () = 0.0;
            cloud->sensor_orientation_.y () = 0.0;
            cloud->sensor_orientation_.z () = 0.0;
          //  listener_.release(frames_);
            return (cloud);
	}

};
