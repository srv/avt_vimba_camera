/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------
 
  File:        VimbaSystem.h

  Description: Definition of class AVT::VmbAPI::VimbaSystem.

-------------------------------------------------------------------------------

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
  NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
  DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED  
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=============================================================================*/

#ifndef AVT_VMBAPI_SYSTEM_H
#define AVT_VMBAPI_SYSTEM_H

#include <vector>

#include <VimbaC/Include/VimbaC.h>
#include <VimbaCPP/Include/VimbaCPPCommon.h>
#include <VimbaCPP/Include/LoggerDefines.h>
#include <VimbaCPP/Include/SharedPointerDefines.h>
#include <VimbaCPP/Include/Interface.h>
#include <VimbaCPP/Include/Camera.h>
#include <VimbaCPP/Include/ICameraFactory.h>
#include <VimbaCPP/Include/ICameraListObserver.h>
#include <VimbaCPP/Include/IInterfaceListObserver.h>

namespace AVT {
namespace VmbAPI {

typedef std::vector<InterfacePtr> InterfacePtrVector;

class VimbaSystem
{
  public:
    //
    // Method:      GetInstance()
    //
    // Purpose:     Returns a reference to the singleton.
    //
    // Parameters:  none
    //
    // Returns:
    //  - VimbaSystem&
    //
    IMEXPORT static VimbaSystem& GetInstance();

    //
    // Method:    QueryVersion()
    //
    // Purpose:   Retrieve the version number of VmbAPI.
    //
    // Parameters:
    //  [out]  VmbVersionInfo_t&   version      Reference to the struct where version information
    //                                          is copied
    //
    // Returns:
    //  - VmbErrorSuccess:       If no error
    //  - VmbErrorStructSize:    The given struct size is not valid for this version of the API
    //  - VmbErrorBadParameter:  "pVersionInfo" is NULL.
    //
    // Details:    This function can be called at anytime, even before the API is
    //             initialized. All other version numbers may be queried via feature access
    //
    IMEXPORT VmbErrorType QueryVersion( VmbVersionInfo_t &version );
    
    //
    // Method:      Startup()
    //
    // Purpose:     Initialize the VmbApi module.
    //
    // Parameters:  none
    //
    // Returns:
    //  - VmbErrorSuccess:       If no error
    //  - VmbErrorInternalFault: An internal fault occurred
    //
    // Details:   On successful return, the API is initialized; this is a necessary call.
    //
    // Internal:
    //  - VimbaSystem initialization (open log files, ...)
    //  - Check system configuration (correct version of Transport Layer modules, ...)
    //  - This method must be called before any other VmbApi function is run.
    //
    IMEXPORT VmbErrorType Startup();

    //
    // Method:    Shutdown()
    //
    // Purpose:   Perform a shutdown on the API module.
    //
    // Parameters: none
    //
    // Returns:
    //  - VmbErrorSuccess:       If no error
    //  - VmbErrorInternalFault: An internal fault occurred
    //
    // Details:   This will free some resources and deallocate all physical resources if applicable.
    //
    IMEXPORT VmbErrorType Shutdown();

    //
    // Method:    GetInterfaces()
    //
    // Purpose:   List all the interfaces currently visible to VmbApi.
    //
    // Parameters:
    //  [out]  InterfacePtrVector& Interfaces            Vector of shared pointer to Interface object
    //
    // Returns:
    //  - VmbErrorSuccess:       If no error
    //  - VmbErrorApiNotStarted: VmbStartup() was not called before the current command
    //  - VmbErrorStructSize:    The given struct size is not valid for this API version
    //  - VmbErrorMoreData:      More data was returned than space was provided
    //
    // Details:     All the interfaces known via a GenTL are listed by this command and filled into the vector provided.
    //              If the vector is not empty, new elements will be appended.
    //              Interfaces may be adapter cards or frame grabber cards, for instance.
    //
    VmbErrorType GetInterfaces( InterfacePtrVector &interfaces );

    //
    // Method:    GetInterfaceByID()
    //
    // Purpose:   Gets a specific interface identified by an ID.
    //
    // Parameters:
    //  [out]  InterfacePtr&        pInterface           Shared pointer to Interface object
    //
    // Returns:
    //  - VmbErrorSuccess:          If no error
    //  - VmbErrorApiNotStarted:    VmbStartup() was not called before the current command
    //  - VmbErrorStructSize:       The given struct size is not valid for this API version
    //  - VmbErrorMoreData:         More data was returned than space was provided
    //
    // Details:     An interface known via a GenTL is listed by this command and filled into the pointer provided.
    //              Interface may be adapter card or frame grabber card, for instance.
    //
    IMEXPORT VmbErrorType GetInterfaceByID( const char *pID, InterfacePtr &pInterface );

    //
    // Method:      OpenInterfaceByID()
    //
    // Purpose:     Open an interface for feature access.
    //
    // Parameters:
    //
    //  [in ]  const char*      pID                 The unique ID of the interface to get
    //  [out]  InterfacePtr&    pInterface          A shared pointer to the interface
    //
    // Returns:
    //
    //  - VmbErrorSuccess:       If no error
    //  - VmbErrorApiNotStarted: VmbStartup() was not called before the current command
    //  - VmbErrorNotFound:      The designated interface cannot be found
    //
    // Details:     An interface can be opened if interface-specific control is required, such as I/O pins
    //              on a frame grabber card. Control is then possible via feature access methods.
    //
    IMEXPORT VmbErrorType OpenInterfaceByID( const char *pID, InterfacePtr &pInterface );

    //
    // Method:    GetCameras()
    //
    // Purpose:   Retrieve a list of all cameras.
    //
    // Parameters:
    //  [out]  CameraPtrVector& rCameras            Vector of shared pointer to Camera object
    //
    // Returns:
    //  - VmbErrorSuccess:       If no error
    //  - VmbErrorApiNotStarted: VmbStartup() was not called before the current command
    //  - VmbErrorStructSize:    The given struct size is not valid for this API version
    //  - VmbErrorMoreData:      More data was returned than space was provided
    //
    // Details:     A camera known via a GenTL is listed by this command and filled into the pointer provided.
    //
    VmbErrorType GetCameras( CameraPtrVector &cameras );

    //
    // Method:    GetCameraByID()
    //
    // Purpose:   Gets a specific camera identified by an ID. The returned camera is still closed.
    //
    // Parameters:
    //  [out]  CameraPtr&           pCamera              Shared pointer to camera object
    //
    // Returns:
    //  - VmbErrorSuccess:          If no error
    //  - VmbErrorApiNotStarted:    VmbStartup() was not called before the current command
    //  - VmbErrorStructSize:       The given struct size is not valid for this API version
    //  - VmbErrorMoreData:         More data was returned than space was provided
    //
    // Details:     A camera known via a GenTL is listed by this command and filled into the pointer provided.
    //              Only static properties of the camera can be fetched until the camera has been opened.
	//				A GigE camera can be identified with its IP address as well.
    //
    IMEXPORT VmbErrorType GetCameraByID( const char *pID, CameraPtr &pCamera );
    
    //
    // Method:      OpenCameraByID()
    //
    // Purpose:     Gets a specific camera identified by an ID. The returned camera is already open.
    //
    // Parameters:
    //
    //  [in ]   const char*         pID                 The unique ID of the camera to get
    //  [in ]   VmbAccessModeType   eAccessMode         The requested access mode
    //  [out]   CameraPtr&          pCamera             A shared pointer to the camera
    //
    // Returns:
    //
    //  - VmbErrorSuccess:       If no error
    //  - VmbErrorApiNotStarted: VmbStartup() was not called before the current command
    //  - VmbErrorNotFound:      The designated interface cannot be found
    //
    // Details:     A camera can be opened if camera-specific control is required, such as I/O pins
    //              on a frame grabber card. Control is then possible via feature access methods.
	//				A GigE camera can be identified with its IP address as well.
    //
    IMEXPORT VmbErrorType OpenCameraByID( const char *pID, VmbAccessModeType eAccessMode, CameraPtr &pCamera );

    //
    // Method:      RegisterCameraListObserver()
    //
    // Purpose:     Registers an instance of camera observer who's CameraListChanged() method gets called
    //              as soon as a camera is plugged in, plugged out or changes its access status
    //
    // Parameters:
    //
    //  [in ]       const ICameraListObserverPtr    &pObserver      A shared pointer to an object derived from ICameraListObserver
    //
    // Returns:
    //
    //  - VmbErrorSuccess:      If no error
    //  - VmbErrorInvalidCall:  If the very same observer is already registered
    //
    IMEXPORT VmbErrorType RegisterCameraListObserver( const ICameraListObserverPtr &pObserver );

    //
    // Method:      UnregisterCameraListObserver()
    //
    // Purpose:     Unregisters a camera observer
    //
    // Parameters:
    //
    //  [in ]       const ICameraListObserverPtr    &pObserver      A shared pointer to an object derived from ICameraListObserver
    //
    // Returns:
    //
    //  - VmbErrorSuccess:      If no error
    //  - VmbErrorNotFound:     If the observer is not registered
    //
    IMEXPORT VmbErrorType UnregisterCameraListObserver( const ICameraListObserverPtr &pObserver );

    //
    // Method:      RegisterInterfaceListObserver()
    //
    // Purpose:     Registers an instance of interface observer whose InterfaceListChanged() method gets called
    //              as soon as an interface is plugged in, plugged out, or changes its access status
    //
    // Parameters:
    //
    //  [in ]       const IInterfaceListObserverPtr    &pObserver      A shared pointer to an object derived from IInterfaceListObserver
    //
    // Returns:
    //
    //  - VmbErrorSuccess:      If no error
    //  - VmbErrorInvalidCall:  If the very same observer is already registered
    //
    IMEXPORT VmbErrorType RegisterInterfaceListObserver( const IInterfaceListObserverPtr &pObserver );

    //
    // Method:      UnregisterInterfaceListObserver()
    //
    // Purpose:     Unregisters an interface observer
    //
    // Parameters:
    //
    //  [in ]       const IInterfaceListObserverPtr    &pObserver      A shared pointer to an object derived from IInterfaceListObserver
    //
    // Returns:
    //
    //  - VmbErrorSuccess:      If no error
    //  - VmbErrorNotFound:     If the observer is not registered
    //
    IMEXPORT VmbErrorType UnregisterInterfaceListObserver( const IInterfaceListObserverPtr &pObserver );

    //
    // Method:      RegisterCameraFactory()
    //
    // Purpose:     Registers an instance of camera factory. When a custom camera factory is registered, all instances of type camera
    //              will be set up accordingly.
    //
    // Parameters:
    //
    //  [in ]       const ICameraFactoryPtr         &cameraFactory  A shared pointer to an object derived from ICameraFactory
    //
    // Returns:
    //
    //  - VmbErrorSuccess:      If no error
    //
    IMEXPORT VmbErrorType RegisterCameraFactory( const ICameraFactoryPtr &cameraFactory );

    //
    // Method:      UnregisterCameraFactory()
    //
    // Purpose:     Unregisters the camera factory. After unregistering the default camera class is used.
    //
    // Returns:
    //
    //  - VmbErrorSuccess:      If no error
    //
    IMEXPORT VmbErrorType UnregisterCameraFactory();

    // Mapping of handle to CameraPtr
    CameraPtr GetCameraPtrByHandle( const VmbHandle_t handle ) const;

    Logger GetLogger() const;

  private:
    // Singleton.
    static VimbaSystem _instance;
    VimbaSystem();
    VimbaSystem( const VimbaSystem& );
    ~VimbaSystem();
    VimbaSystem& operator=( const VimbaSystem& system );
    
    struct Impl;
    Impl *m_pImpl;    

    IMEXPORT VmbErrorType GetCameras( CameraPtr *pCameras, VmbUint32_t &size );
    IMEXPORT VmbErrorType GetInterfaces( InterfacePtr *pInterfaces, VmbUint32_t &size );
};

#include <VimbaCPP/Include/VimbaSystem.hpp>

}} // namespace AVT::VmbAPI
#endif
