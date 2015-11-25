/*
Copyright (C) 2012 Steven Hickson

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
//#pragma once


//Irgendwas Definedß? im Build?
//#ifndef __PCL_IO_Libfreenect_GRABBER__
//#define __PCL_IO_Libfreenect_GRABBER__
#include <pcl/pcl_config.h>
#include <a.out.h>
#include <pcl/io/eigen.h>
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <deque>

#include <pcl/common/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/exceptions.h>
#include <iostream>
#include <signal.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <iostream>
#include <Eigen/Core>
#include "Kinect_helper.h"
#include <pthread.h>

/*
#include <pcl/io/eigen.h>
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <string>
#include <deque>
#include <pcl/common/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/exceptions.h>
#include <iostream>

#include <assert.h>
#include <windows.h>
#include <vector>
#include <algorithm>
#include <objbase.h>
#include "Kinect.h"

#include <opencv2/opencv.hpp>
*/


namespace pcl
{
	struct PointXYZ;
	struct PointXYZRGB;
	struct PointXYZRGBA;
	struct PointXYZI;

/*
	class KinectData {
	public:
	//	KinectData() { };
		KinectData(	const libfreenect2::Frame &image_, const	libfreenect2::Frame &depth_, const PointCloud<PointXYZRGBA> &cloud_)
		{
 			image = image_;
 			depth = depth_;
			cloud = cloud_;
		};
		cv::Mat getColor(){
			libfreenect2::Frame * rgb = &image;
			cv::Mat tmp(rgb->height, rgb->width, CV_8UC4, rgb->data);
			cv::Mat r = tmp.clone();
			return std::move(r);
		}

		cv::Mat getDepth(){
			libfreenect2::Frame * depth = &depth;
			cv::Mat tmp(depth->height, depth->width, CV_8UC4, depth->data);
			cv::Mat r = tmp.clone();
			listener_.release(frames_);
			return std::move(r);
		}

		std::pair<cv::Mat, cv::Mat> getDepthRgb(){
			libfreenect2::Frame * depth = &depth;
			libfreenect2::Frame * rgb = &image;
			registration_->apply(rgb, depth, &undistorted_, &registered_);
			cv::Mat tmp_depth(undistorted_.height, undistorted_.width, CV_8UC4, undistorted_.data);
			cv::Mat tmp_color(registered_.height, registered_.width, CV_8UC4, registered_.data);
			cv::Mat r = tmp_color.clone();
			cv::Mat d = tmp_depth.clone();
			return std::move(std::pair<cv::Mat, cv::Mat>(r,d));
		}

		pcl::PointCloud<pcl::PointXYZRGBA> cloud;
		libfreenect2::Frame  image;
		libfreenect2::Frame  depth;

	};*/

	/** \brief Grabber for OpenNI devices (i.e., Primesense PSDK, Microsoft Kinect, Asus XTion Pro/Live)
	* \author Nico Blodow <blodow@cs.tum.edu>, Suat Gedikli <gedikli@willowgarage.com>
	* \ingroup io
	*/


	template <typename T> class PointCloud;
	class MatDepth : public cv::Mat { }; //I have to use this to get around the assuming code in registerCallback in grabber.h

	class PCL_EXPORTS Libfreenect2Grabber : public Grabber
	{
	public:
		typedef boost::shared_ptr<Libfreenect2Grabber> Ptr;
		typedef boost::shared_ptr<const Libfreenect2Grabber> ConstPtr;

		typedef enum
		{
			Libfreenect_Default_Mode = 0, // VGA@30Hz
			Libfreenect_SXGA_15Hz = 1    // Need to fill the rest of this up
		} Mode;

		//define callback signature typedefs
		/*	cv::Mat getColor() */
		typedef void (sig_cb_libfreenect_image) (const boost::shared_ptr<cv::Mat> &);
		/* cv::Mat getDepth() */
		typedef void (sig_cb_libfreenect_depth_image) (const MatDepth &);
		/*std::pair<cv::Mat, cv::Mat> getDepthRgb()*/
		typedef void (sig_cb_libfreenect_image_depth_image) (const boost::shared_ptr<cv::Mat> &, const MatDepth &, float);

		typedef void (sig_cb_libfreenect_point_cloud_rgba) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>> &);

	//	typedef void (sig_cb_libfreenect_all_data) (const boost::shared_ptr<const KinectData> &);
		/*typedef void (sig_cb_microsoft_ir_image) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&);
		typedef void (sig_cb_microsoft_point_cloud) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);
		typedef void (sig_cb_microsoft_point_cloud_rgb) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >&);
		typedef void (sig_cb_microsoft_point_cloud_i) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&);*/

		Libfreenect2Grabber ();
		//const Mode& depth_mode = OpenNI_Default_Mode,
		//const Mode& image_mode = OpenNI_Default_Mode);

		/** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
		virtual ~Libfreenect2Grabber () throw ();

		/** \brief Start the data acquisition. */
		virtual void
			start ();

		/** \brief Stop the data acquisition. */
		virtual void
			stop ();

		/** \brief Check if the data acquisition is still running. */
		virtual bool
			isRunning () const;

		virtual std::string
			getName () const;

		/** \brief Obtain the number of frames per second (FPS). */
		virtual float
			getFramesPerSecond () const;

		/** \brief Get a boost shared pointer to the \ref OpenNIDevice object. */
		/*inline boost::shared_ptr<Microsoft2Grabber>
		getDevice () const;*/


		//Kinect Camera Settings
    //Freenect Camera Settings
    ////TODO: In Freenect umwandeln
		libfreenect2::Registration * registration_ = NULL; // das?
		//ICoordinateMapper*      m_pCoordinateMapper;
		bool CameraSettingsSupported;

//	void GetPointCloudFromData(const boost::shared_ptr<cv::Mat> &img, const MatDepth &depth, boost::shared_ptr<PointCloud<PointXYZRGBA>> &cloud, bool alignToColor, bool preregistered) const;

		//These should not be called except within the thread by the KinectCapture class process manager
		void ProcessThreadInternal();

		void SetLargeCloud() {
			//m_largeCloud = true;
		}

		void SetNormalCloud() {
		//m_largeCloud = false;
		}
 private:
	 void prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p);
	 cv::Mat getColor(libfreenect2::Frame * colorFrameRef);
	 cv::Mat getDepth(libfreenect2::Frame * depthFrameRef);

	 std::pair<cv::Mat, cv::Mat> getDepthRgb(libfreenect2::Frame * colorFrameRef, libfreenect2::Frame *depthFrameRef);

	protected:
        void getCloud(const boost::shared_ptr<cv::Mat>&, const pcl::MatDepth&, boost::shared_ptr<PointCloud<PointXYZRGB>> &cloud) const;

		libfreenect2::Freenect2 freenect2_;
		libfreenect2::PacketPipeline * pipeline_ = NULL;
		libfreenect2::Frame activDepthFrame;
		libfreenect2::Frame activImageFrame;

		libfreenect2::FrameMap frames_;
		libfreenect2::Frame undistorted_/*DepthFrame*/, registered_/*DepthFrame?*/, big_mat_ /*ColorFrame*/;
		Eigen::Matrix<float,512,1> colmap;
		Eigen::Matrix<float,424,1> rowmap;
		std::string serial_;
		int map_[512 * 424];


		boost::signals2::signal<sig_cb_libfreenect_image>* image_signal_;
		boost::signals2::signal<sig_cb_libfreenect_depth_image>* depth_image_signal_;
		boost::signals2::signal<sig_cb_libfreenect_image_depth_image>* image_depth_image_signal_;
		boost::signals2::signal<sig_cb_libfreenect_point_cloud_rgba>* point_cloud_rgba_signal_;
	//	boost::signals2::signal<sig_cb_libfreenect_all_data>* all_data_signal_;
		/*boost::signals2::signal<sig_cb_microsoft_ir_image>* ir_image_signal_;
		boost::signals2::signal<sig_cb_microsoft_point_cloud>* point_cloud_signal_;
		boost::signals2::signal<sig_cb_microsoft_point_cloud_i>* point_cloud_i_signal_;
		boost::signals2::signal<sig_cb_microsoft_point_cloud_rgob>* point_cloud_rgb_signal_;
		*/
		Synchronizer<boost::shared_ptr<cv::Mat>, MatDepth> rgb_sync_;

		bool m_depthStarted, m_videoStarted, m_audioStarted, m_infraredStarted, m_person, m_preregistered;

    //TODO: In Freenect umwandeln
    // Current Kinect
		libfreenect2::Freenect2Device * dev_ = NULL;
//		IKinectSensor*          m_pKinectSensor;


    ////TODO: In Freenect umwandeln
		//IColorFrameReader*      m_pColorFrameReader;
	//	IMultiSourceFrameReader* m_pMultiSourceFrameReader;
		libfreenect2::SyncMultiFrameListener listener_;


	//	static const int        cColorWidth  = 1920;
		//	static const int        cColorHeight = 1080;
	//		static const int        cDepthWidth  = 512;
	//		static const int        cDepthHeight = 424;
	//		cv::Size m_colorSize, m_depthSize;
	//		RGBQUAD* m_pColorRGBX;
	//		UINT16 *m_pDepthBuffer;
		//TODO WAS THAT?
		//	ColorSpacePoint *m_pColorCoordinates;
	//		CameraSpacePoint *m_pCameraSpacePoints;
	//		cv::Mat m_colorImage, m_depthImage;
#define COLOR_PIXEL_TYPE CV_8UC4
#define DEPTH_PIXEL_TYPE CV_16UC1
// BRAUCH ICH DAS?
	//		bool m_largeCloud;

        pthread_t pKinectThread;

	//	HANDLE hStopEvent, hKinectThread, hDepthMutex, hColorMutex;
	/*	WAITABLE_HANDLE hFrameEvent;
		bool m_depthUpdated, m_colorUpdated, m_infraredUpdated, m_skeletonUpdated;
		typedef long LONGLONG;
		typedef int INT64;
		LONGLONG m_rgbTime, m_depthTime, m_infraredTime;
		INT64 timestep;
*/
		//boost::mutex m_depthMutex, m_colorMutex, m_infraredMutex;

		void Release();
		void GetNextFrame();
		void FrameArrived();
		void DepthFrameArrived(libfreenect2::Frame * depthFrameRef);
		void ColorFrameArrived(libfreenect2::Frame * colorFrameRef);
		//void BodyIndexFrameArrived(IBodyIndexFrameReference* pBodyIndexFrameReference);
		bool GetCameraSettings();

		void imageDepthImageCallback(const boost::shared_ptr<cv::Mat> &image,const MatDepth &depth_image);
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> convertToXYZRGBAPointCloud (const boost::shared_ptr<cv::Mat> &image, const MatDepth &depth_image) const;
		/** \brief Convert a Depth + RGB image pair to a pcl::PointCloud<PointT>
		* \param[in] image the RGB image to convert
		* \param[in] depth_image the depth image to convert
		*/
		/*template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
		convertToXYZRGBPointCloud (const boost::shared_ptr<openni_wrapper::Image> &image,
		const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image) const;*/
	};

}
