//ROS
#include <ros/ros.h>

//PROJECT
#include <avt_vimba_camera/vimba_ros.h>

namespace avt_vimba_ros {

VimbaROS::VimbaROS(ros::NodeHandle nh, ros::NodeHandle nhp) : vimba_(AVT::VmbAPI::VimbaSystem::GetInstance())
{

  initApi();
}

// Translates Vimba error codes to readable error messages
std::string VimbaROS::errorCodeToMessage( VmbErrorType error )
{
  std::map<VmbErrorType, std::string>::const_iterator iter = error_code_to_message_.find( error );
  if ( error_code_to_message_.end() != iter )
  {
    return iter->second;
  }
  return "Unsupported error code passed.";
}

int VimbaROS::getWidth()
{
  return (int)width_;
}

int VimbaROS::getHeight()
{
  return (int)height_;
}

VmbPixelFormatType VimbaROS::getPixelFormat()
{
  return (VmbPixelFormatType)pixel_format_;
}

void VimbaROS::listAvailableCameras(void)
{
	std::string name;
	AVT::VmbAPI::CameraPtrVector cameras;

	if (VmbErrorSuccess == vimba_.Startup())
	{
		if (VmbErrorSuccess == vimba_.GetCameras(cameras))
		{
			for (AVT::VmbAPI::CameraPtrVector::iterator iter = cameras.begin();
				  cameras.end() != iter;
				  ++iter)
			{
				if (VmbErrorSuccess == (*iter)->GetName(name))
				{
					ROS_INFO_STREAM("[AVT_Vimba_ROS]: Found camera: " << name);
				}
			}
		}
	}
}

std::string VimbaROS::interfaceToString( VmbInterfaceType interfaceType )
{
    switch ( interfaceType )
    {
        case VmbInterfaceFirewire: return "FireWire";
            break;
        case VmbInterfaceEthernet: return "GigE";
            break;
        case VmbInterfaceUsb: return "USB";
            break;
        default: return "Unknown";
    }
}

AVT::VmbAPI::CameraPtr VimbaROS::openCamera(std::string id_str)
{
	// Details:   The ID might be one of the following:
	//						"IP:169.254.12.13", 
	//						"MAC:000f31000001",
  //            or a plain serial number: "1234567890".
    
	AVT::VmbAPI::CameraPtr camera;	

	if (VmbErrorSuccess == vimba_.OpenCameraByID(id_str.c_str(),VmbAccessModeFull,camera))
	{
		std::string cam_id,cam_name,cam_model,cam_sn,cam_int,cam_int_type_str;
		VmbInterfaceType cam_int_type;
		camera->GetID(cam_id);
		camera->GetName(cam_name);
		camera->GetModel(cam_model);
		camera->GetSerialNumber(cam_sn);
		camera->GetInterfaceID(cam_int);
		camera->GetInterfaceType(cam_int_type);
		cam_int_type_str = interfaceToString(cam_int_type);

		ROS_INFO_STREAM("[AVT_Vimba_ROS]: Opened camera with" 
			<< "\n\t\t * Name:      " << cam_name
			<< "\n\t\t * Model:     " << cam_model
			<< "\n\t\t * ID:        " << cam_id
			<< "\n\t\t * S/N:       " << cam_sn
			<< "\n\t\t * Itf. ID:   " << cam_int
			<< "\n\t\t * Itf. Type: " << cam_int_type);
    printAllCameraFeatures(camera);
	}
	return camera;
}

void VimbaROS::printAllCameraFeatures(AVT::VmbAPI::CameraPtr camera)
{
	VmbErrorType err;
	AVT::VmbAPI::FeaturePtrVector features; 

  std::string strCamID;                                                   // The ID of our camera

	// The static details of a feature
	std::string strName;                                                    // The name of the feature
	std::string strDisplayName;                                             // The display name of the feature
	std::string strTooltip;                                                 // A short description of the feature
	std::string strDescription;                                             // A long description of the feature
	std::string strCategory;                                                // A category to group features
	std::string strSFNCNamespace;                                           // The Standard Feature Naming Convention namespace
	std::string strUnit;                                                    // The measurement unit of the value
	VmbFeatureDataType eType;                                               // The data type of the feature

	// The changeable value of a feature
	VmbInt64_t  nValue;                                                     // An int value
	double      fValue;                                                     // A float value
	std::string strValue;                                                   // A string value
	bool        bValue;                                                     // A bool value

  std::stringstream strError;

	err = camera->GetFeatures( features );                         // Fetch all features of our cam
  if ( VmbErrorSuccess == err )
  {
      // Query all static details as well as the value of all fetched features and print them out.
      for (   AVT::VmbAPI::FeaturePtrVector::const_iterator iter = features.begin();
              features.end() != iter;
              ++iter )
      {
          err = (*iter)->GetName( strName );
          if ( VmbErrorSuccess != err )
          {
              strError << "[Could not get feature Name. Error code: " << err << "]";
              strName.assign( strError.str() );
          }

          err = (*iter)->GetDisplayName( strDisplayName );
          if ( VmbErrorSuccess != err )
          {
              strError << "[Could not get feature Display Name. Error code: " << err << "]";
              strDisplayName.assign( strError.str() );
          }

          err = (*iter)->GetToolTip( strTooltip );
          if ( VmbErrorSuccess != err )
          {
              strError << "[Could not get feature Tooltip. Error code: " << err << "]";
              strTooltip.assign( strError.str() );
          }

          err = (*iter)->GetDescription( strDescription );
          if ( VmbErrorSuccess != err )
          {
              strError << "[Could not get feature Description. Error code: " << err << "]";
              strDescription.assign( strError.str() );
          }

          err = (*iter)->GetCategory( strCategory );
          if ( VmbErrorSuccess != err )
          {
              strError << "[Could not get feature Category. Error code: " << err << "]";
              strCategory.assign( strError.str() );
          }

          err = (*iter)->GetSFNCNamespace( strSFNCNamespace );
          if ( VmbErrorSuccess != err )
          {
              strError << "[Could not get feature SNFC Namespace. Error code: " << err << "]";
              strSFNCNamespace.assign( strError.str() );
          }

          err = (*iter)->GetUnit( strUnit );
          if ( VmbErrorSuccess != err )
          {
              strError << "[Could not get feature Unit. Error code: " << err << "]";
              strUnit.assign( strError.str() );
          }

          std::cout << "/// Feature Name: " << strName << std::endl;
          std::cout << "/// Display Name: " << strDisplayName << std::endl;
          std::cout << "/// Tooltip: " << strTooltip << std::endl;
          std::cout << "/// Description: " << strDescription << std::endl;
          std::cout << "/// SNFC Namespace: " << strSFNCNamespace << std::endl;
          std::cout << "/// Value: ";

          err = (*iter)->GetDataType( eType );
          if ( VmbErrorSuccess != err )
          {
              std::cout << "[Could not get feature Data Type. Error code: " << err << "]" << std::endl;
          }
          else
          {
              switch ( eType )
              {
                  case VmbFeatureDataBool:
                      err = (*iter)->GetValue( bValue );
                      if ( VmbErrorSuccess == err )
                      {
                          std::cout << bValue << std::endl;
                      }
                      break;
                  case VmbFeatureDataEnum:
                      err = (*iter)->GetValue( strValue );
                      if ( VmbErrorSuccess == err )
                      {
                          std::cout << strValue << std::endl;
                      }
                      break;
                  case VmbFeatureDataFloat:
                      err = (*iter)->GetValue( fValue );
                      {
                          std::cout << fValue << std::endl;
                      }
                      break;
                  case VmbFeatureDataInt:
                      err = (*iter)->GetValue( nValue );
                      {
                          std::cout << nValue << std::endl;
                      }
                      break;
                  case VmbFeatureDataString:
                      err = (*iter)->GetValue( strValue );
                      {
                          std::cout << strValue << std::endl;
                      }
                      break;
                  case VmbFeatureDataCommand:
                  default:
                      std::cout << "[None]" << std::endl;
                      break;

                  if ( VmbErrorSuccess != err )
                  {
                      std::cout << "Could not get feature value. Error code: " << err << std::endl;
                  }
              }
          }
          
          std::cout << std::endl;
      }
  }
  else
  {
      std::cout << "Could not get features. Error code: " << err << std::endl;
  }
}





void VimbaROS::initApi(void)
{
  error_code_to_message_[ VmbErrorSuccess ] =           "Success.";
  error_code_to_message_[ VmbErrorInternalFault ] =     "Unexpected fault in VmbApi or driver.";    
  error_code_to_message_[ VmbErrorApiNotStarted ] =     "API not started.";     
  error_code_to_message_[ VmbErrorNotFound ] =          "Not found.";
  error_code_to_message_[ VmbErrorBadHandle ] =         "Invalid handle ";
  error_code_to_message_[ VmbErrorDeviceNotOpen ] =     "Device not open.";
  error_code_to_message_[ VmbErrorInvalidAccess ] =     "Invalid access.";
  error_code_to_message_[ VmbErrorBadParameter ] =      "Bad parameter.";
  error_code_to_message_[ VmbErrorStructSize ] =        "Wrong DLL version.";
  error_code_to_message_[ VmbErrorMoreData ] =          "More data returned than memory provided.";
  error_code_to_message_[ VmbErrorWrongType ] =         "Wrong type.";
  error_code_to_message_[ VmbErrorInvalidValue ] =      "Invalid value.";
  error_code_to_message_[ VmbErrorTimeout ] =           "Timeout.";
  error_code_to_message_[ VmbErrorOther ] =             "TL error.";
  error_code_to_message_[ VmbErrorResources ] =         "Resource not available.";
  error_code_to_message_[ VmbErrorInvalidCall ] =       "Invalid call.";
  error_code_to_message_[ VmbErrorNoTL ] =              "TL not loaded.";
  error_code_to_message_[ VmbErrorNotImplemented ] =    "Not implemented.";
  error_code_to_message_[ VmbErrorNotSupported ] =      "Not supported.";

  if (VmbErrorSuccess == vimba_.Startup())
  {
    ROS_INFO_STREAM("[AVT_Vimba_ROS]: AVT Vimba System initialized successfully");
    listAvailableCameras();
  }else{
    ROS_ERROR_STREAM("[AVT_Vimba_ROS]: Could not start Vimba system.");
  }
}

};
