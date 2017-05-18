/*
 *  $Id$
 */
#include "TU/Image++.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

namespace TU
{
inline void
increment(int* value)
{
    ++(*value);
}

class StereoFlow
{
  private:
    typedef sensor_msgs::Image				Msg;
    typedef message_filters::sync_policies::ExactTime<Msg, Msg>
							ExactPolicy;
    typedef message_filters::sync_policies::ApproximateTime<Msg, Msg>
							ApproxPolicy;
    typedef message_filters::Synchronizer<ExactPolicy>	ExactSync;
    typedef message_filters::Synchronizer<ApproxPolicy>	ApproxSync;

    class KernelBase
    {
      public:
	virtual ~KernelBase()						{}

	virtual void	setImages(const sensor_msgs::Image& msgL,
				  const sensor_msgs::Image& msgR)	= 0;
	virtual void	saveHeaders(std::ostream& out)		const	= 0;
	virtual void	saveData(std::ostream& out)		const	= 0;
    };

    template <class T>
    class Kernel : public KernelBase
    {
      public:
	Kernel(size_t nimages)	:_images(nimages)			{}
	virtual ~Kernel()						{}

	virtual void
	setImages(const sensor_msgs::Image& msgL, const sensor_msgs::Image& msgR)
	{
	    _images[0].resize(reinterpret_cast<T*>(
				  const_cast<u_char*>(&msgL.data[0])),
			      msgL.height, msgL.width);
	    _images[1].resize(reinterpret_cast<T*>(
				  const_cast<u_char*>(&msgR.data[0])),
			      msgR.height, msgR.width);
	}
	
	virtual void
	saveHeaders(std::ostream& out) const
	{
	    out << 'M' << _images.size() << std::endl;
	    for (const auto& image : _images)
		image.saveHeader(out);
	}

	virtual void
	saveData(std::ostream& out) const
	{
	    for (const auto& image : _images)
		image.saveData(out);
	}
	
      private:
	Array<Image<T> >	_images;
    };

  public:
    StereoFlow(const std::string& transport)
	:_nreceivedL(0), _nreceivedR(0), _nreceivedAll(0), _kernel(nullptr)
    {
      // Read local parameters
	ros::NodeHandle local_nh("~");
	bool		autosize;
	local_nh.param("autosize", autosize, true);
    
      // Resolve topic names
	ros::NodeHandle	nh;
	std::string	topicL = ros::names::clean("/multisense/left/" +
						   nh.resolveName("image"));
	std::string	topicR = ros::names::clean("/multisense/right/" +
						   nh.resolveName("image"));
      //ROS_WARN("Subscribing to:\n\t* %s\n\t* %s",
      //	 topicL.c_str(), topicR.c_str());

      // Subscribe to two input topics.
	image_transport::ImageTransport	it(nh);
	_subL.subscribe(it, topicL, 1, transport);
	_subR.subscribe(it, topicR, 1, transport);

      // Complain every 30s if the topics appear unsynchronized
	_subL.registerCallback(boost::bind(increment, &_nreceivedL));
	_subR.registerCallback(boost::bind(increment, &_nreceivedR));
	_synced_timer = nh.createWallTimer(
			    ros::WallDuration(15.0),
			    boost::bind(&StereoFlow::checkInputsSynchronized,
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
	    _approx_sync->registerCallback(boost::bind(&StereoFlow::imageCb,
						       this, _1, _2));
	}
	else
	{
	    _exact_sync.reset(new ExactSync(ExactPolicy(_queue_size),
					    _subL, _subR));
	    _exact_sync->registerCallback(boost::bind(&StereoFlow::imageCb,
						      this, _1, _2));
	}
    }

    ~StereoFlow()
    {
	delete _kernel;
    }

    void
    imageCb(const sensor_msgs::ImageConstPtr& msgL,
	    const sensor_msgs::ImageConstPtr& msgR)
    {
	std::cerr << msgL->header.stamp.sec << '.'
		  << msgL->header.stamp.nsec/1000000 << '.'
		  << msgL->header.stamp.nsec/1000%1000 << std::endl;

	++_nreceivedAll; // For error checking

	if (!_kernel)
	{
	    const auto&	encoding = msgL->encoding;
	    if (encoding == sensor_msgs::image_encodings::MONO8)
		_kernel = new Kernel<u_char>(2);
	    else if (encoding == sensor_msgs::image_encodings::MONO16)
		_kernel = new Kernel<short>(2);
	    else if (encoding == sensor_msgs::image_encodings::RGB8)
		_kernel = new Kernel<RGB>(2);
	    else if (encoding == sensor_msgs::image_encodings::RGBA8)
		_kernel = new Kernel<RGBA>(2);
	    else if (encoding == sensor_msgs::image_encodings::BGR8)
		_kernel = new Kernel<BGR>(2);
	    else if (encoding == sensor_msgs::image_encodings::BGRA8)
		_kernel = new Kernel<BGRA>(2);
	    else if (encoding == sensor_msgs::image_encodings::RGB8)
		_kernel = new Kernel<RGB>(2);
	    else
		throw std::runtime_error("Unknown pixel format!");

	    _kernel->setImages(*msgL, *msgR);
	    _kernel->saveHeaders(std::cout);
	}
	else
	    _kernel->setImages(*msgL, *msgR);
	_kernel->saveData(std::cout);
    }

    void
    checkInputsSynchronized() const
    {
	int	threshold = 2 * _nreceivedAll;
	if (_nreceivedL >= threshold || _nreceivedR >= threshold)
	{
	    ROS_WARN("[stereo_flow] Low number of synchronized left/right/disparity triplets received.\n"
		     "Left images received:      %d (topic '%s')\n"
		     "Right images received:     %d (topic '%s')\n"
		     "Synchronized pairs: %d\n"
		     "Possible issues:\n"
		     "\t* stereo_image_proc is not running.\n"
		     "\t  Does `rosnode info %s` show any connections?\n"
		     "\t* The cameras are not synchronized.\n"
		     "\t  Try restarting stereo_flow with parameter _approximate_sync:=True\n"
		     "\t* The network is too slow. One or more images are dropped from each triplet.\n"
		     "\t  Try restarting stereo_flow, increasing parameter 'queue_size' (currently %d)",
		     _nreceivedL, _subL.getTopic().c_str(),
		     _nreceivedR, _subR.getTopic().c_str(),
		     _nreceivedAll, ros::this_node::getName().c_str(),
		     _queue_size);
	}
    }

  private:
    image_transport::SubscriberFilter	_subL, _subR;
    std::shared_ptr<ExactSync>		_exact_sync;
    std::shared_ptr<ApproxSync>		_approx_sync;
    int					_queue_size;
  
    ros::WallTimer			_synced_timer;
    int					_nreceivedL, _nreceivedR, _nreceivedAll;

    KernelBase				*_kernel;
};
}

int
main(int argc, char *argv[])
{
    ros::init(argc, argv,
	      "stereo_flow", ros::init_options::AnonymousName);
    if (ros::names::remap("image") == "/image_raw")
    {
	ROS_WARN("There is a delay between when the camera drivers publish the raw images and "
		 "when stereo_image_proc publishes the computed point cloud. stereo_flow "
		 "may fail to synchronize these topics without a large queue_size.");
    }

    std::string		transport = (argc > 1 ? argv[1] : "raw");
    TU::StereoFlow	flow(transport);
  
    ros::spin();

    return 0;
}
