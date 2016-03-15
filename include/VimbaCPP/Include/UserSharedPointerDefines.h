#ifndef AVT_VMBAPI_USERSHAREDPOINTER_H
#define AVT_VMBAPI_USERSHAREDPOINTER_H

#include "..\..\..\..\VimbaNET\Include\NetPointer.h"
    
namespace AVT {
namespace VmbAPINET {

ref class Camera;
ref class Interface;
ref class Frame;
ref class Feature;
ref class AncillaryData;

};
};

namespace AVT {
namespace VmbAPI {

// Set the calls for your implementation of the shared pointer functions
// a) Declaration
// b) Reset with argument
// c) Reset without argument
// d) == operator
// e) NULL test
// f) Access to underlying raw pointer

// a) This is the define for a declaration.
#define SP_DECL( T )			NetPointer<T>
// b) This is the define for setting an existing shared pointer.
#define SP_SET( sp, rawPtr )	sp.Reset( rawPtr )
// c) This is the define for resetting without an argument to decrease the ref count.
#define SP_RESET( sp )			sp.Reset()
// d) This is the define for the equal operator. Shared pointers are usually considered equal when the raw pointers point to the same address.
#define SP_ISEQUAL( sp1, sp2 )  sp1.IsEqualTo(sp2)
// e) This is the define for the NULL check.
#define SP_ISNULL( sp )         sp.IsNull()
// f) This is the define for the raw pointer access. This is usually accomplished through the dereferencing operator (->).
#define SP_ACCESS( sp )         sp.AccessNative()

class Camera;
typedef NetPointer<Camera, AVT::VmbAPINET::Camera> CameraPtr;

class Interface;
typedef NetPointer<Interface, AVT::VmbAPINET::Interface> InterfacePtr;

class Feature;
typedef NetPointer<Feature, AVT::VmbAPINET::Feature> FeaturePtr;
	
class FeatureContainer;
typedef SP_DECL( FeatureContainer ) FeatureContainerPtr;

class IFeatureObserver;
typedef SP_DECL( IFeatureObserver ) IFeatureObserverPtr;

class Frame;
typedef NetPointer<Frame, AVT::VmbAPINET::Frame> FramePtr;

class FrameHandler;
typedef SP_DECL( FrameHandler ) FrameHandlerPtr;

class IFrameObserver;
typedef SP_DECL( IFrameObserver ) IFrameObserverPtr;

class AncillaryData;
typedef NetPointer<AncillaryData, AVT::VmbAPINET::AncillaryData> AncillaryDataPtr;
typedef NetPointer<AncillaryData, AVT::VmbAPINET::AncillaryData> ConstAncillaryDataPtr;

class ICameraFactory;
typedef SP_DECL( ICameraFactory) ICameraFactoryPtr;

class IInterfaceListObserver;
typedef SP_DECL( IInterfaceListObserver ) IInterfaceListObserverPtr;

class ICameraListObserver;
typedef SP_DECL( ICameraListObserver ) ICameraListObserverPtr;

class Mutex;
typedef SP_DECL( Mutex ) MutexPtr;

class BasicLockable;
typedef SP_DECL( BasicLockable ) BasicLockablePtr;

}}

#include "..\..\..\..\VimbaNET\Include\NetCamera.h"
#include "..\..\..\..\VimbaNET\Include\NetInterface.h"
#include "..\..\..\..\VimbaNET\Include\NetFrame.h"
#include "..\..\..\..\VimbaNET\Include\NetFeature.h"
#include "..\..\..\..\VimbaNET\Include\NetAncillaryData.h"

#endif /* AVT_VMBAPI_USERSHAREDPOINTER_H */
