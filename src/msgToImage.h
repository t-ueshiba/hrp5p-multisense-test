/*
 *  $Id$
 */
#include "TU/Image++.h"

namespace TU
{
class CaptureAndSave
{
  private:
    class KernelBase
    {
      public:
	virtual ~KernelBase()						{}

	virtual std::ostream&	saveHeaders(std::ostream& out)	const	= 0;
    };

    template <class T>
    class Kernel : public KernelBase
    {
      public:
	Kernel(const Array<CAMERA*>& cameras)				;
	virtual ~Kernel()						{}
	
	virtual std::ostream&	saveHeaders(std::ostream& out)	const	;
	
      private:
	const Array<CAMERA*>&		_cameras;
	mutable Array<Image<T> >	_images;
    };
    
  public:
    CaptureAndSave(const Array<CAMERA*>& cameras)
	:_kernel(0)				{ setFormat(cameras); }
    ~CaptureAndSave()				{ delete _kernel; }
    
    void		setFormat(const Array<CAMERA*>& cameras)	;
    std::ostream&	saveHeaders(std::ostream& out) const
			{
			    return _kernel->saveHeaders(out);
			}
    
  private:
    KernelBase*		_kernel;
};

template <class CAMERA> template <class T>
CaptureAndSave<CAMERA>::Kernel<T>::Kernel(const Array<CAMERA*>& cameras)
    :_cameras(cameras), _images(_cameras.size())
{
    auto	image = _images.begin();
    for (const auto camera : _cameras)
    {
	image->resize(camera->height(), camera->width());
	++image;
    }
}
    
template <class CAMERA> template <class T> std::ostream&
CaptureAndSave<CAMERA>::Kernel<T>::saveHeaders(std::ostream& out) const
{
    out << 'M' << _images.size() << std::endl;
    for (const auto& image : _images)
	image.saveHeader(out);

    return out;
}
    
}
