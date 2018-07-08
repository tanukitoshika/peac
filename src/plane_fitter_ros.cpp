#include <ros/ros.h>

#pragma warning(disable: 4996)
#pragma warning(disable: 4819)
#define _CRT_SECURE_NO_WARNINGS

#include <sensor_msgs/PointCloud2.h>

//#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>

#include <opencv2/opencv.hpp>

#include "AHCPlaneFitter.hpp"

using ahc::utils::Timer;

ros::Publisher pub;

// pcl::PointCloud interface for our ahc::PlaneFitter
template<class PointT>
struct OrganizedImage3D {
    const pcl::PointCloud<PointT> &cloud;
	//NOTE: pcl::PointCloud from OpenNI uses meter as unit,
	//while ahc::PlaneFitter assumes mm as unit!!!
	const double unitScaleFactor;

	OrganizedImage3D(const pcl::PointCloud<PointT>& c) : cloud(c), unitScaleFactor(1000) {}
	int width() const { return cloud.width; }
	int height() const { return cloud.height; }
	bool get(const int row, const int col, double& x, double& y, double& z) const {
		const PointT& pt=cloud.at(col,row);
		x=pt.x; y=pt.y; z=pt.z;
		return pcl_isnan(z)==0; //return false if current depth is NaN
	}
};
typedef OrganizedImage3D<pcl::PointXYZRGBA> RGBDImage;
typedef ahc::PlaneFitter<RGBDImage> PlaneFitter;

class MainLoop
{
protected:
	PlaneFitter pf;
	cv::Mat rgb, seg;
	bool done;

public:
	MainLoop () : done(false) {}

	void plane_fitter(const sensor_msgs::PointCloud2ConstPtr& input);
	void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);

	void loopSubscriber(ros::NodeHandle* nh)
	{
		ROS_INFO("Initializing Subscribers");
		ros::Subscriber sub = nh->subscribe ("input", 1, &MainLoop::cloud_cb, this);
	}

	//process a new frame of point cloud
    void onNewCloud (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{
		//fill RGB
	    if(rgb.empty() || rgb.rows!=cloud->height || rgb.cols!=cloud->width) {
		    rgb.create(cloud->height, cloud->width, CV_8UC3);
			seg.create(cloud->height, cloud->width, CV_8UC3);
		}
		for(int i=0; i<(int)cloud->height; ++i) {
		    for(int j=0; j<(int)cloud->width; ++j) {
				const pcl::PointXYZRGBA& p=cloud->at(j,i);
				if(!pcl_isnan(p.z)) {
					rgb.at<cv::Vec3b>(i,j)=cv::Vec3b(p.b,p.g,p.r);
				} else {
					rgb.at<cv::Vec3b>(i,j)=cv::Vec3b(255,255,255);//whiten invalid area
				}
			}
		}

		//run PlaneFitter on the current frame of point cloud
		RGBDImage rgbd(*cloud);
		Timer timer(1000);
		timer.tic();
		pf.run(&rgbd, 0, &seg);
		double process_ms=timer.toc();

		//blend segmentation with rgb
		cv::cvtColor(seg,seg,CV_RGB2BGR);
		seg=(rgb+seg)/2.0;
		
		//show frame rate
		std::stringstream stext;
		stext<<"Frame Rate: "<<(1000.0/process_ms)<<"Hz";
		cv::putText(seg, stext.str(), cv::Point(15,15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255,1));

		cv::imshow("rgb", rgb);
		cv::imshow("seg", seg);
	}

	//start the main loop
	void run (const sensor_msgs::PointCloud2ConstPtr& input)
	{
		ROS_INFO("Converting the sensor_msgs/PointCloud2 data to pcl/PointCloud");
        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::fromROSMsg (*input, *cloud);

		cv::namedWindow("rgb");
		cv::namedWindow("seg");
		cv::namedWindow("control");//, cv::WINDOW_NORMAL);

		MainLoop::onNewCloud(cloud);

		int mergeMSETol=(int)pf.params.stdTol_merge,
			minSupport=(int)pf.minSupport,
			doRefine=(int)pf.doRefine;

		cv::createTrackbar("epsilon","control", &mergeMSETol, (int)pf.params.stdTol_merge*2);
		cv::createTrackbar("T_{NUM}","control", &minSupport, pf.minSupport*5);
		cv::createTrackbar("Refine On","control", &doRefine, 1);
		cv::createTrackbar("windowHeight","control", &pf.windowHeight, 2*pf.windowHeight);
		cv::createTrackbar("windowWidth","control", &pf.windowWidth, 2*pf.windowWidth);

		pf.params.stdTol_merge=(double)mergeMSETol;
		pf.minSupport=minSupport;
		pf.doRefine=doRefine!=0;
		onKey(cv::waitKey(1000));

        pub.publish(cloud);
	}
    
    	//handle keyboard commands
	void onKey(const unsigned char key)
	{
		static bool stop=false;
		switch(key) {
		case 'q':
			this->done=true;
			break;
		}
	}
};

int main (int argc, char** argv)
{
    bool done = false;
	MainLoop loop;

    // Initialize ROS
    ros::init (argc, argv, "plane_fitter");
    ros::NodeHandle nh;

    // Set ROS param
    ros::param::set("dist_th", 0.1);

    // // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("input", 1, &MainLoop::run, &loop);
	//loop.loopSubscriber(&nh);

    // Create a ROS publisher for the model coefficients
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA> > ("plane_fitter", 1);
    // Spin
    ros::spin ();
}
