/*
 *  $Id$
 */
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include <boost/thread.hpp>

#ifdef HAVE_GTK
#  include <gtk/gtk.h>

// Platform-specific workaround for #3026: image_view doesn't close when
// closing image window. On platforms using GTK+ we connect this to the
// window's "destroy" event so that image_view exits.
static void
destroy(GtkWidget *widget, gpointer data)
{
    ros::shutdown();
}
#endif

inline void
increment(int* value)
{
    ++(*value);
}

class StereoView
{
  private:
    typedef sensor_msgs::Image				Msg;
    typedef message_filters::sync_policies::ExactTime<Msg, Msg>
							ExactPolicy;
    typedef message_filters::sync_policies::ApproximateTime<Msg, Msg>
							ApproxPolicy;
    typedef message_filters::Synchronizer<ExactPolicy>	ExactSync;
    typedef message_filters::Synchronizer<ApproxPolicy>	ApproxSync;

  private:
    image_transport::SubscriberFilter	_subL, _subR;
    std::shared_ptr<ExactSync>		_exact_sync;
    std::shared_ptr<ApproxSync>		_approx_sync;
    int					_queue_size;
  
    cv::Mat				_imgL, _imgR;
    boost::mutex			_mutex;
  
    ros::WallTimer			_synced_timer;
    int					_nreceivedL, _nreceivedR, _nreceivedAll;

  public:
    StereoView(const std::string& transport)
	:_nreceivedL(0), _nreceivedR(0), _nreceivedAll(0)
    {
      // Read local parameters
	ros::NodeHandle local_nh("~");
	bool		autosize;
	local_nh.param("autosize", autosize, true);
    
      // Do GUI window setup
	int	flags = (autosize ? cv::WND_PROP_AUTOSIZE : 0);
	cv::namedWindow("left",  flags);
	cv::namedWindow("right", flags);
#if CV_MAJOR_VERSION == 2
#  ifdef HAVE_GTK
	g_signal_connect(GTK_WIDGET( cvGetWindowHandle("left") ),
			 "destroy", G_CALLBACK(destroy), NULL);
	g_signal_connect(GTK_WIDGET( cvGetWindowHandle("right") ),
			 "destroy", G_CALLBACK(destroy), NULL);
#  endif
	cvStartWindowThread();
#endif

      // Resolve topic names
	ros::NodeHandle	nh;
	std::string	topicL = ros::names::clean("/multisense/left/" +
						   nh.resolveName("image"));
	std::string	topicR = ros::names::clean("/multisense/right/" +
						   nh.resolveName("image"));
	ROS_INFO("Subscribing to:\n\t* %s\n\t* %s",
		 topicL.c_str(), topicR.c_str());

      // Subscribe to two input topics.
	image_transport::ImageTransport	it(nh);
	_subL.subscribe(it, topicL, 1, transport);
	_subR.subscribe(it, topicR, 1, transport);

      // Complain every 30s if the topics appear unsynchronized
	_subL.registerCallback(boost::bind(increment, &_nreceivedL));
	_subR.registerCallback(boost::bind(increment, &_nreceivedR));
	_synced_timer = nh.createWallTimer(
			    ros::WallDuration(15.0),
			    boost::bind(&StereoView::checkInputsSynchronized,
					this));

      // Synchronize input topics.
      // Optionally do approximate synchronization.
	local_nh.param("queue_size", _queue_size, 5);
	bool	approx;
	local_nh.param("approximate_sync", approx, false);
	if (approx)
	{
	    _approx_sync.reset(new ApproxSync(ApproxPolicy(_queue_size),
					      _subL, _subR));
	    _approx_sync->registerCallback(boost::bind(&StereoView::imageCb,
						       this, _1, _2));
	}
	else
	{
	    _exact_sync.reset(new ExactSync(ExactPolicy(_queue_size),
					    _subL, _subR));
	    _exact_sync->registerCallback(boost::bind(&StereoView::imageCb,
						      this, _1, _2));
	}
    }

    ~StereoView()
    {
	cv::destroyAllWindows();
    }

    void
    imageCb(const sensor_msgs::ImageConstPtr& msgL,
	    const sensor_msgs::ImageConstPtr& msgR)
    {
	std::cout << msgL->header.stamp.sec << '.'
		  << msgL->header.stamp.nsec/1000000 << '.'
		  << msgL->header.stamp.nsec/1000%1000 << std::endl;

	++_nreceivedAll; // For error checking
	
	_mutex.lock();

      // May want to view raw bayer data
	if (msgL->encoding.find("bayer") != std::string::npos)
	    boost::const_pointer_cast<Msg>(msgL)->encoding = "mono8";
	if (msgR->encoding.find("bayer") != std::string::npos)
	    boost::const_pointer_cast<Msg>(msgR)->encoding = "mono8";

      // Hang on to image data for sake of mouseCb
	try
	{
	    _imgL = cv_bridge::toCvShare(msgL, "bgr8")->image;
	    _imgR = cv_bridge::toCvShare(msgR, "bgr8")->image;
	}
	catch (cv_bridge::Exception& e)
	{
	    ROS_ERROR("Unable to convert one of '%s' or '%s' to 'bgr8'",
		      msgL->encoding.c_str(), msgR->encoding.c_str());
	}

      // Must release the mutex before calling cv::imshow,
      // or can deadlock against OpenCV's window mutex.
	_mutex.unlock();
	if (!_imgL.empty())
	    cv::imshow("left", _imgL);
	if (!_imgR.empty())
	    cv::imshow("right", _imgR);
    }

    void
    checkInputsSynchronized() const
    {
	int	threshold = 2 * _nreceivedAll;
	if (_nreceivedL >= threshold || _nreceivedR >= threshold)
	{
	    ROS_WARN("[stereo_view] Low number of synchronized left/right/disparity triplets received.\n"
		     "Left images received:      %d (topic '%s')\n"
		     "Right images received:     %d (topic '%s')\n"
		     "Synchronized pairs: %d\n"
		     "Possible issues:\n"
		     "\t* stereo_image_proc is not running.\n"
		     "\t  Does `rosnode info %s` show any connections?\n"
		     "\t* The cameras are not synchronized.\n"
		     "\t  Try restarting stereo_view with parameter _approximate_sync:=True\n"
		     "\t* The network is too slow. One or more images are dropped from each triplet.\n"
		     "\t  Try restarting stereo_view, increasing parameter 'queue_size' (currently %d)",
		     _nreceivedL, _subL.getTopic().c_str(),
		     _nreceivedR, _subR.getTopic().c_str(),
		     _nreceivedAll, ros::this_node::getName().c_str(),
		     _queue_size);
	}
    }
};

int
main(int argc, char *argv[])
{
    ros::init(argc, argv,
	      "stereo_view", ros::init_options::AnonymousName);
    if (ros::names::remap("image") == "/image_raw")
    {
	ROS_WARN("There is a delay between when the camera drivers publish the raw images and "
		 "when stereo_image_proc publishes the computed point cloud. stereo_view "
		 "may fail to synchronize these topics without a large queue_size.");
    }

    std::string	transport = (argc > 1 ? argv[1] : "raw");
    StereoView	view(transport);
  
    ros::spin();

    return 0;
}
