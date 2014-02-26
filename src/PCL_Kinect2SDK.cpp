#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

#include "Microsoft_grabber2.h"
#include <pcl/visualization/cloud_viewer.h>
/*#include <FaceTrackLib.h>
#include <KinectInteraction.h>
#include <NuiKinectFusionApi.h>
#include <NuiKinectFusionDepthProcessor.h>
#include <NuiKinectFusionVolume.h>*/

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace pcl;
using namespace cv;

class SimpleMicrosoftViewer
{
public:
	SimpleMicrosoftViewer () : viewer(new pcl::visualization::PCLVisualizer ("PCL Microsoft Viewer")), normals(new pcl::PointCloud<pcl::Normal>), sharedCloud(new pcl::PointCloud<pcl::PointXYZRGBA>), first(false), update(false) {}

	void cloud_cb_ (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&data)
	{
		// estimate normals
		/*imshow("image", *(data->image));
		imshow("depth", data->depth);
		waitKey(2);*/
		if(!data->empty()) {
			normalMutex.lock();
			copyPointCloud(*data,*sharedCloud);
			/*pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
			ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
			ne.setMaxDepthChangeFactor(0.02f);
			ne.setNormalSmoothingSize(10.0f);
			ne.setInputCloud(sharedCloud);
			ne.compute(*normals);*/
			update = true;
			normalMutex.unlock();
		}
	}

	void image_cb_ (const Mat *image) {
		imshow("image",*image);
		waitKey(1);
	}

	void depth_cb_ (const MatDepth &depth) {
		imshow("depth",depth);
		waitKey(1);
	}

	void run ()
	{
		// create a new grabber for OpenNI devices
		pcl::Grabber* my_interface = new pcl::Microsoft2Grabber();

		// make callback function from member function
		//boost::function<void (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&)> f =
		//boost::bind (&SimpleMicrosoftViewer::cloud_cb_, this, _1);
		boost::function<void (const Mat*)> f2 =
		boost::bind (&SimpleMicrosoftViewer::image_cb_, this, _1);
		boost::function<void (const MatDepth&)> f3 =
		boost::bind (&SimpleMicrosoftViewer::depth_cb_, this, _1);

		//my_interface->registerCallback (f);
		my_interface->registerCallback (f2);
		my_interface->registerCallback (f3);

		//viewer.setBackgroundColor(0.0, 0.0, 0.5);
		my_interface->start ();
		Sleep(30);
		//while(1)
		//	boost::this_thread::sleep (boost::posix_time::seconds (1));
		while (!viewer->wasStopped())
		{
			normalMutex.lock();
			if(update) {
				//viewer->removePointCloud("cloud");
				viewer->removePointCloud("original");
				viewer->addPointCloud(sharedCloud,"original");
				//viewer->addPointCloudNormals<pcl::PointXYZRGBA,pcl::Normal>(sharedCloud, normals);
				update = false;
			}
			viewer->spinOnce();
			normalMutex.unlock();
		}

		my_interface->stop ();
	}

	void run2() {
		// estimate normals
		PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>());
		pcl::io::loadPCDFile("C:/Users/Steve/Documents/test.pcd",*cloud);
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
		ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
		ne.setMaxDepthChangeFactor(0.02f);
		ne.setNormalSmoothingSize(10.0f);
		ne.setInputCloud(cloud);
		ne.compute(*normals);

		// visualize normals
		//viewer.addPointCloud<PointXYZRGBA>(cloud,"original");
		/*viewer->addPointCloudNormals<pcl::PointXYZRGBA,pcl::Normal>(cloud, normals);
		while (!viewer->wasStopped())
			viewer->spinOnce(100);*/
	}

	boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> sharedCloud;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	bool first, update;
	boost::mutex normalMutex;
};

int
	main (int argc, char** argv)
{
	PointCloud<PointXYZ> depth;
	PointCloud<PointXYZRGB> color;
	PointCloud<PointXYZRGB> cloud;
	try {
		SimpleMicrosoftViewer v;
		v.run();
	} catch (pcl::PCLException e) {
		cout << e.detailedMessage() << endl;
	} catch (std::exception &e) {
		cout << e.what() << endl;
	}
	cin.get();
	return (0);
}