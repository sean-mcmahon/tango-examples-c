/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tango-gl/conversions.h>
#include <tango_support_api.h>

#include <rgb-depth-sync/rgb_depth_sync_application.h>

namespace rgb_depth_sync {


void SynchronizationApplication::writeCameraIntrinsics2Text(const TangoCameraIntrinsics tango_camera_intrinsics_) {

    std::ofstream intrinsicsFile ("/sdcard/Download/Tango_Camera_Intrinsics.txt");
    if (intrinsicsFile.is_open()) {
        intrinsicsFile << "calibration_type:\n";
        switch(tango_camera_intrinsics_.calibration_type) {
            case TANGO_CALIBRATION_EQUIDISTANT             : intrinsicsFile << "TANGO_CALIBRATION_EQUIDISTANT\n"; break;
            case TANGO_CALIBRATION_POLYNOMIAL_3_PARAMETERS : intrinsicsFile << "TANGO_CALIBRATION_POLYNOMIAL_3_PARAMETERS\n"; break;
            default: intrinsicsFile << "Unkown/Invalid\n";
        }
        intrinsicsFile << "TangoCameraId:\n";
        switch(tango_camera_intrinsics_.camera_id) {
            case TANGO_CAMERA_COLOR   : intrinsicsFile << "TANGO_CAMERA_COLOR\n"; break;
            case TANGO_CAMERA_RGBIR   : intrinsicsFile << "TANGO_CAMERA_RGBIR\n"; break;
            case TANGO_CAMERA_FISHEYE : intrinsicsFile << "TANGO_CAMERA_FISHEYE\n"; break;
            case TANGO_CAMERA_DEPTH   : intrinsicsFile << "TANGO_CAMERA_DEPTH,\n"; break;
            case TANGO_MAX_CAMERA_ID  : intrinsicsFile << "TANGO_MAX_CAMERA_ID\n"; break;
            default: intrinsicsFile << "Unkown/Invalid\n";
        }
        intrinsicsFile << "cx:\n" <<std::fixed << std::setprecision(8) << tango_camera_intrinsics_.cx << "\n";
        intrinsicsFile << "cy:\n" <<std::fixed << std::setprecision(8) << tango_camera_intrinsics_.cy << "\n";
        intrinsicsFile << "distortion:\n";
        for (int i=0; i < 5; i++) {
            intrinsicsFile <<std::fixed << std::setprecision(8) << tango_camera_intrinsics_.distortion[i] << ", ";
        }
        intrinsicsFile << "\nfx:\n" <<std::fixed << std::setprecision(8) << tango_camera_intrinsics_.fx << "\n";
        intrinsicsFile << "fy:\n" <<std::fixed << std::setprecision(8) << tango_camera_intrinsics_.fy << "\n";
        intrinsicsFile << "height:\n" << tango_camera_intrinsics_.height << "\n";
        intrinsicsFile << "width:\n" << tango_camera_intrinsics_.width;

    }
    else {
        LOGE("writeCameraIntrinsics2Text: Could not open Tango_Camera_Intrinsics.txt");
    }
}

// This function will route callbacks to our application object via the context
// parameter.
// @param context Will be a pointer to a SynchronizationApplication instance  on
// which to call callbacks.
// @param xyz_ij The point cloud to pass on.
void OnXYZijAvailableRouter(void* context, const TangoXYZij* xyz_ij) {
  SynchronizationApplication* app =
      static_cast<SynchronizationApplication*>(context);
  app->OnXYZijAvailable(xyz_ij);
}

void OnFrameAvailableRouter(void* context, TangoCameraId id, const TangoImageBuffer* buffer) {
    SynchronizationApplication* app =
            static_cast<SynchronizationApplication*>(context);
    app->OnFrameAvailable(buffer);
}

void SynchronizationApplication::OnXYZijAvailable(const TangoXYZij* xyz_ij) {
  // We'll just update the point cloud associated with our depth image.
  TangoSupport_updatePointCloud(point_cloud_manager_, xyz_ij);
}

void SynchronizationApplication::OnFrameAvailable(const TangoImageBuffer* buffer) {
    // We'll just update the color image.... fuckers
    TangoSupport_updateImageBuffer(color_image_manager_, buffer);
}

SynchronizationApplication::SynchronizationApplication()
    : color_image_(),
      depth_image_(),
      main_scene_(),
      // We'll store the fixed transform between the opengl frame convention.
      // (Y-up, X-right) and tango frame convention. (Z-up, X-right).
      OW_T_SS_(tango_gl::conversions::opengl_world_T_tango_world()),
      gpu_upsample_(false) {}

SynchronizationApplication::~SynchronizationApplication() {
  if (tango_config_) {
    TangoConfig_free(tango_config_);
  }
  TangoSupport_freePointCloudManager(point_cloud_manager_);
  point_cloud_manager_ = nullptr;
  TangoSupport_freeImageBufferManager(color_image_manager_);
  color_image_manager_ = nullptr;
    if (myfile.is_open()) {
        saving_to_file_ = false;
        myfile.close();
        std::ofstream TS_count;
        TS_count.open("/sdcard/Download/my_file_save_iterations.txt");
        TS_count << num_write_iterations;
        TS_count.close();
        num_write_iterations = 0;
    }
}

bool SynchronizationApplication::CheckTangoVersion(JNIEnv* env,
                                                   jobject activity,
                                                   int min_tango_version) {
  // Check that we have the minimum required version of Tango.
  int version;
  TangoErrorType err = TangoSupport_GetTangoVersion(env, activity, &version);

  return err == TANGO_SUCCESS && version >= min_tango_version;
}

void SynchronizationApplication::OnTangoServiceConnected(JNIEnv* env,
                                                         jobject binder) {
  TangoErrorType ret = TangoService_setBinder(env, binder);
  if (ret != TANGO_SUCCESS) {
    LOGE(
        "SynchronizationApplication: Failed to set Tango service binder with"
        "error code: %d",
        ret);
  }
}

bool SynchronizationApplication::TangoSetupConfig() {
  SetDepthAlphaValue(0.0);
  SetGPUUpsample(false);

  if (tango_config_ != nullptr) {
    return true;
  }

  // Here, we'll configure the service to run in the way we'd want. For this
  // application, we'll start from the default configuration
  // (TANGO_CONFIG_DEFAULT). This enables basic motion tracking capabilities.
  tango_config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
  if (tango_config_ == nullptr) {
    return false;
  }

  // In addition to motion tracking, however, we want to run with depth so that
  // we can sync Image data with Depth Data. As such, we're going to set an
  // additional flag "config_enable_depth" to true.
  TangoErrorType err =
      TangoConfig_setBool(tango_config_, "config_enable_depth", true);
    TangoErrorType err2 =
            TangoConfig_setBool(tango_config_, "config_enable_color_camera", true);
  if (err != TANGO_SUCCESS || err2 != TANGO_SUCCESS) {
    LOGE("Failed to enable depth or color camera.");
    return false;
  }

  // We also need to enable the color camera in order to get RGB frame
  // callbacks.
//  err = TangoConfig_setBool(tango_config_, "config_enable_color_camera", true);
//  if (err != TANGO_SUCCESS) {
//    LOGE(
//        "Failed to set 'enable_color_camera' configuration flag with error"
//        " code: %d",
//        err);
//    return false;
//  }

  // Note that it's super important for AR applications that we enable low
  // latency imu integration so that we have pose information available as
  // quickly as possible. Without setting this flag, you'll often receive
  // invalid poses when calling GetPoseAtTime for an image.
  err = TangoConfig_setBool(tango_config_,
                            "config_enable_low_latency_imu_integration", true);
  if (err != TANGO_SUCCESS) {
    LOGE("Failed to enable low latency imu integration.");
    return false;
  }

  // Use the tango_config to set up the PointCloudManager before we connect
  // the callbacks.
  if (point_cloud_manager_ == nullptr) {
    int32_t max_point_cloud_elements;
    err = TangoConfig_getInt32(tango_config_, "max_point_cloud_elements",
                               &max_point_cloud_elements);
    if (err != TANGO_SUCCESS) {
      LOGE("Failed to query maximum number of point cloud elements.");
      return false;
    }

    err = TangoSupport_createPointCloudManager(max_point_cloud_elements,
                                               &point_cloud_manager_);
    if (err != TANGO_SUCCESS) {
      return false;
    }
  }
    if (color_image_manager_ == nullptr) {
        err = TangoSupport_createImageBufferManager(TANGO_HAL_PIXEL_FORMAT_YCrCb_420_SP, image_width_,
                                                    image_height_, &color_image_manager_);
        if (err != TANGO_SUCCESS)
        {
            LOGE("SynchronizationApplication: Failed to create image buffer manager.");
            return false;
        }
    }
    myfile.open("/sdcard/Download/Two_ts_all_data.bin");
    if (myfile.is_open())
    {
        LOGI("TangoSetUpConfig: Successful opennng of myfile");
    }
    else {
        LOGE("TangoSetUpConfig: Unsuccessful openning of myfile for data recording");
        return false;
    }

  return true;
}

bool SynchronizationApplication::TangoConnectTexture() {
  // The Tango service allows you to connect an OpenGL texture directly to its
  // RGB and fisheye cameras. This is the most efficient way of receiving
  // images from the service because it avoids copies. You get access to the
  // graphic buffer directly. As we're interested in rendering the color image
  // in our render loop, we'll be polling for the color image as needed.
  TangoErrorType err = TangoService_connectTextureId(
      TANGO_CAMERA_COLOR, color_image_.GetTextureId(), this, nullptr);
  return err == TANGO_SUCCESS;
}

bool SynchronizationApplication::TangoConnectCallbacks() {
  // We're interested in only one callback for this application. We need to be
  // notified when we receive depth information in order to support measuring
  // 3D points. For both pose and color camera information, we'll be polling.
  // The render loop will drive the rate at which we need color images and all
  // our poses will be driven by timestamps. As such, we'll use GetPoseAtTime.
  TangoErrorType depth_ret =
      TangoService_connectOnXYZijAvailable(OnXYZijAvailableRouter);
  TangoErrorType color_ret =
      TangoService_connectOnFrameAvailable(TANGO_CAMERA_COLOR, this, OnFrameAvailableRouter);
  return (depth_ret == TANGO_SUCCESS) && (color_ret == TANGO_SUCCESS);
}

bool SynchronizationApplication::TangoConnect() {
  // Here, we'll connect to the TangoService and set up to run. Note that we're
  // passing in a pointer to ourselves as the context which will be passed back
  // in our callbacks.
  TangoErrorType ret = TangoService_connect(this, tango_config_);
  if (ret != TANGO_SUCCESS) {
    LOGE("SynchronizationApplication: Failed to connect to the Tango service.");
    return false;
  }
  return true;
}
bool SynchronizationApplication::TangoSetIntrinsicsAndExtrinsics() {
  // Get the intrinsics for the color camera and pass them on to the depth
  // image. We need these to know how to project the point cloud into the color
  // camera frame.
  TangoCameraIntrinsics color_camera_intrinsics;
  TangoErrorType err = TangoService_getCameraIntrinsics(
      TANGO_CAMERA_COLOR, &color_camera_intrinsics);
  if (err != TANGO_SUCCESS) {
    LOGE(
        "SynchronizationApplication: Failed to get the intrinsics for the color"
        "camera.");
    return false;
  }

  writeCameraIntrinsics2Text(color_camera_intrinsics);
  image_width_ = color_camera_intrinsics.width;
  image_height_ =  color_camera_intrinsics.height;
  depth_image_.SetCameraIntrinsics(color_camera_intrinsics);
  main_scene_.SetCameraIntrinsics(color_camera_intrinsics);

  // Initialize TangoSupport context.
  TangoSupport_initialize(TangoService_getPoseAtTime);
  return true;
}

void SynchronizationApplication::TangoDisconnect() {
  TangoService_disconnect();
    if (myfile.is_open() && saving_to_file_==true) {
        saving_to_file_ = false;
        myfile.close();
        std::ofstream TS_count;
        TS_count.open("/sdcard/Download/my_file_save_iterations.txt");
        TS_count << num_write_iterations;
        TS_count.close();
        num_write_iterations = 0;
    }
}

void SynchronizationApplication::InitializeGLContent() {
  depth_image_.InitializeGL();
  color_image_.InitializeGL();
  fisheye_image_.InitializeGL();
  main_scene_.InitializeGL();
}

void SynchronizationApplication::SetViewPort(int width, int height) {
  screen_width_ = static_cast<float>(width);
  screen_height_ = static_cast<float>(height);
  main_scene_.SetupViewPort(width, height);
}

void SynchronizationApplication::Render() {
  double color_timestamp = 0.0;
  double depth_timestamp = 0.0;
  bool new_point_cloud = false;
  bool new_pointsTwo = false;
  double timediff = 0.99;

  // Sean - Define what motion is requested
  TangoCoordinateFramePair frames_of_reference_;
    frames_of_reference_.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
    frames_of_reference_.target = TANGO_COORDINATE_FRAME_DEVICE;

  TangoSupport_getLatestPointCloudAndNewDataFlag(point_cloud_manager_,
                                                 &render_buffer_, &new_point_cloud);


  depth_timestamp = render_buffer_->timestamp;
  // We need to make sure that we update the texture associated with the color
  // image.
  successful_color_image_retreval = true;
  if (TangoService_updateTexture(TANGO_CAMERA_COLOR, &color_timestamp) !=
      TANGO_SUCCESS) {
    LOGE("SynchronizationApplication: Failed to update color image to texture.");
      successful_color_image_retreval = false;
  }

    // Get latest colour image manger and buffer.
    TangoSupport_getLatestImageBufferAndNewDataFlag(color_image_manager_, &color_image_buffer_,&new_pointsTwo);

  // In the following code, we define t0 as the depth timestamp and t1 as the
  // color camera timestamp.
    if (color_image_buffer_==nullptr) {
        LOGE("color image buffer is a null pointer!");
    }

    if (color_buffer_list_.size() <= 10) { // I want 5 elements in my list
        color_buffer_list_.push_back(*color_image_buffer_);
//        LOGI("Push_back on color image list; list size is %d", color_buffer_list_.size());
        image_list_iterator_ = color_buffer_list_.begin();
    }
    else {
//        LOGI("List elements are:");
//        for (std::list<int>::iterator testIterator =color_buffer_list_.begin(); testIterator!=color_buffer_list_.end(); testIterator++ ) {
//            LOGI("-: %d,",(*testIterator).timestamp);
//        }
        if (image_list_iterator_ == color_buffer_list_.end()) {
            image_list_iterator_ = color_buffer_list_.begin();
//            LOGI("List iterator has reached end of list, reassigning to list.begin(). distance %d", std::distance(color_buffer_list_.begin(), image_list_iterator_));
        }
//        LOGI("Erasing element... distance %d", std::distance(color_buffer_list_.begin(), image_list_iterator_));
        image_list_iterator_ = color_buffer_list_.erase(image_list_iterator_);
//        LOGI("erased element at distance %d ; list size %d",std::distance(color_buffer_list_.begin(), image_list_iterator_), color_buffer_list_.size() );
        color_buffer_list_.insert(image_list_iterator_,*color_image_buffer_);
//        LOGI("Inserted element at distance %d ; list size %d",std::distance(color_buffer_list_.begin(), image_list_iterator_), color_buffer_list_.size() );
//        image_list_iterator_++;
    }

  // Calculate the relative pose from color camera frame at timestamp
  // color_timestamp t1 and depth
  // camera frame at depth_timestamp t0.
  TangoPoseData pose_color_image_t1_T_depth_image_t0;
  if (TangoSupport_calculateRelativePose(
          color_timestamp, TANGO_COORDINATE_FRAME_CAMERA_COLOR, depth_timestamp,
          TANGO_COORDINATE_FRAME_CAMERA_DEPTH,
          &pose_color_image_t1_T_depth_image_t0) != TANGO_SUCCESS) {
    LOGE(
        "SynchronizationApplication: Could not find a valid relative pose at "
        "time for color and "
        " depth cameras.");
    return;
  }


  // The Color Camera frame at timestamp t0 with respect to Depth
  // Camera frame at timestamp t1.
  glm::mat4 color_image_t1_T_depth_image_t0 =
      util::GetMatrixFromPose(&pose_color_image_t1_T_depth_image_t0);

      if (gpu_upsample_) {
        depth_image_.RenderDepthToTexture(color_image_t1_T_depth_image_t0,
                                          render_buffer_,
                                          new_point_cloud);
      } else {
        depth_image_.UpdateAndUpsampleDepth(color_image_t1_T_depth_image_t0,
                                            render_buffer_);
      }

    // Save the colour image (color_image_buffer_ (struct)), depth image
    // color_image_buffer_.depth_map_buffer_ (float vector), point cloud render_buffer_
    // and pose info pose_color_image_t1_T_depth_image_t0.
    // Save ONE instance of TangoCameraIntrinsics per recording
    // Save any instance of TangoEvent
    //TangoEvent* myEventInstance;

    // this isn't technically every second, as depth_timestamp only updates when a new point cloud
    // has been recorded.
//    if (render_buffer_->color_image == nullptr) {
//        LOGE("No color_image");
//    }
//    else {
//        LOGI("%f", render_buffer_->color_image->timestamp);
//        }
//    if (color_image_buffer_ == nullptr) {
//        LOGE("No color image buffer");
//    }
//    else {
//        LOGI("%f", color_image_buffer_->timestamp);
//    }
//    LOGI("Render Buffer Color Image Info timestamp: %f", color_image_buffer_->timestamp);
    if ((color_timestamp - time_buffer_ >= timediff)) {
        // Select Ideal Color Image (closest timestamp to depth image).Overwite colo_image_buffer
        *color_image_buffer_ = getImageClosestToTS(color_buffer_list_, render_buffer_->timestamp);

        TangoPoseData device_pose_on_image_retreval_;
        /* This is a totally gross way to do this. I should put this somewhere below with the writing of the
         * data (same if condition). But I dislike the idea of replicating the code down there for each
         * condition even more.
         */
        if (TangoService_getPoseAtTime(color_image_buffer_->timestamp, frames_of_reference_,
                                       &device_pose_on_image_retreval_) != TANGO_SUCCESS) {
            LOGE("SynchronizationApplication: Failed to get pose at colour image capture; timestamp %f",
                 color_image_buffer_->timestamp);
        }

        // Save stuff
        //LOGI("Timestamp: %f with time buffer %f ", depth_timestamp, time_buffer_);
        if (saving_to_file_ == true && myfile.is_open() && device_pose_on_image_retreval_.status_code==TANGO_POSE_VALID) {
            std::vector<float> my_depth_image_buffer_ = depth_image_.getDepthMapBuffer();
            if (my_depth_image_buffer_.empty()) {
                LOGE("SynchronizationApplication::Render - Depth image buffer empty! @ Timestep %f", render_buffer_->timestamp);
            }
            const int post_status_string_length = 24;
            char mypose_status_ [post_status_string_length];
            // this pose is set on the status of the latest PC being available.
            switch (device_pose_on_image_retreval_.status_code) {
                case TANGO_POSE_INITIALIZING: strcpy(mypose_status_,"TANGO_POSE_INITIALIZING"); break;
                case TANGO_POSE_INVALID     : strcpy(mypose_status_,"TANGO_POSE_INVALID-----"); break;
                case TANGO_POSE_UNKNOWN     : strcpy(mypose_status_,"TANGO_POSE_UNKNOWN-----"); break;
                case TANGO_POSE_VALID       : strcpy(mypose_status_,"TANGO_POSE_VALID-------"); break;
                default : strcpy(mypose_status_,"Invalid_status_code----");
            }

             // no new point cloud, sync latest color image
                myfile.write(reinterpret_cast<const char *>(&color_image_buffer_->data[0]),
                             std::streamsize(image_width_ * (image_height_ + image_height_ /
                                                                             2))); // YUV 420 SP format, sizes are height*1.5, width
                myfile.write(reinterpret_cast<const char *>(&color_image_buffer_->timestamp),
                             sizeof(double));
                myfile.write((char *) &(my_depth_image_buffer_[0]),
                             my_depth_image_buffer_.size() * sizeof(float));
                myfile.write(reinterpret_cast<const char *>(&render_buffer_->timestamp),
                             sizeof(double));
                myfile.write(
                        reinterpret_cast<const char *>(&device_pose_on_image_retreval_.accuracy),
                        sizeof(float));
                myfile.write(
                        reinterpret_cast<const char *>(&device_pose_on_image_retreval_.orientation[0]),
                        std::streamsize(4 * sizeof(double)));
                myfile.write(reinterpret_cast<const char *>(&mypose_status_),
                             std::streamsize(sizeof(char) * post_status_string_length));
                myfile.write(
                        reinterpret_cast<const char *>(&device_pose_on_image_retreval_.timestamp),
                        sizeof(double));
                myfile.write(
                        reinterpret_cast<const char *>(&device_pose_on_image_retreval_.translation[0]),
                        std::streamsize(3 * sizeof(double)));
                num_write_iterations++;

                //saving_to_file_=false;
//            LOGI("Saved example file, timestamp: %f, sizeof: %zu, image size %d", render_buffer_->timestamp,sizeof(float), std::streamsize(image_width_*(image_height_+image_height_/2)));
//            LOGI("ColorCameraIntinsics. height: %d, width: %d, depth: %d, and uint8_t size: %zu",image_height_, image_width_, image_depth_,
//                 sizeof(uint8_t) );
                LOGI("ColorImageBuffer. height: %d, width: %d, depth: %d,buffer timestamp %f, and Format: %04x (0x11 = YCbCr_420_SP)",
                     color_image_buffer_->width, color_image_buffer_->height, image_depth_,
                     color_image_buffer_->timestamp, color_image_buffer_->format);
                LOGI("First few values of color_image_buffer_->data are: %u, %u, %u, %u, %u ",
                     color_image_buffer_->data[0], color_image_buffer_->data[220395],
                     color_image_buffer_->data[220405], color_image_buffer_->data[220400],
                     color_image_buffer_->data[230400]); //230400] );
                LOGI("First values of depth_image_buffer are: %f,%f,%f,%f,%f ",
                     my_depth_image_buffer_[50], my_depth_image_buffer_[220395],
                     my_depth_image_buffer_[220405], my_depth_image_buffer_[220400],
                     my_depth_image_buffer_[230400]);
//            LOGI("Size of variable types. Float %lu, double %lu , int %lu ", sizeof(float), sizeof(double),
//                 sizeof(int));
                LOGI("Pose_on_color_update Orentation %f, %f, %f, %f. Translation x,y,z %f, %f, %f ",
                     device_pose_on_image_retreval_.orientation[0],
                     device_pose_on_image_retreval_.orientation[1],
                     device_pose_on_image_retreval_.orientation[2],
                     device_pose_on_image_retreval_.orientation[3],
                     device_pose_on_image_retreval_.translation[0],
                     device_pose_on_image_retreval_.translation[1],
                     device_pose_on_image_retreval_.translation[2]);
                LOGI("Pose status: %s, Pose accuracy %f and timestamp %f ", mypose_status_,
                     device_pose_on_image_retreval_.accuracy,
                     device_pose_on_image_retreval_.timestamp);
        }
        else if (!myfile.is_open() && saving_to_file_==true) {
            LOGE("Save file is not open!");
        }

        time_buffer_ = depth_timestamp;
    }

    if (num_write_iterations >= 2 && saving_to_file_==true) {
        LOGI("Closing myfile...");
        saving_to_file_ = false;
        myfile.close();
        std::ofstream TS_count;
        TS_count.open("/sdcard/Download/my_file_save_iterations.txt");
        TS_count << num_write_iterations;
        TS_count.close();
        if (myfile.is_open()) {
            LOGE("Unsuccessful close of myfile");
        }
        if (autoReset== true) {
            LOGE("Re-openning myfile, check filenames");
            myfile.open("/sdcard/Download/Two_ts_all_data.bin");
            saving_to_file_ = true;
            num_write_iterations = 0;
        }
    }
      main_scene_.Render(color_image_.GetTextureId(),
                         depth_image_.GetTextureId());
}

    /*void SynchronizationApplication::RenderFisheye() {
        main_scene_.Render(fisheye_image_.GetTextureId());
    }*/

void SynchronizationApplication::SetDepthAlphaValue(float alpha) {
  main_scene_.SetDepthAlphaValue(alpha);
}

void SynchronizationApplication::SetGPUUpsample(bool on) { gpu_upsample_ = on; }

TangoImageBuffer SynchronizationApplication::getImageClosestToTS( std::list<TangoImageBuffer> image_list_, const double depth_timestamp) {
    TangoImageBuffer buffer, closest_image_;
    closest_image_ = image_list_.front();
//    LOGI("Depth timestamp value %f", depth_timestamp);
    for (std::list<TangoImageBuffer>::iterator myIterator = image_list_.begin(); myIterator!=image_list_.end(); myIterator++ ) {
        buffer = *myIterator;
//        LOGI("Difference is color TS is %f and difference is %f",buffer.timestamp, std::abs(buffer.timestamp - depth_timestamp));
        if (std::abs(buffer.timestamp - depth_timestamp) < std::abs(closest_image_.timestamp - depth_timestamp)) {
            closest_image_ = buffer;
            }
    }
    LOGI("Closes color image is %f, with diff %f", closest_image_.timestamp, std::abs(closest_image_.timestamp - depth_timestamp));
    return closest_image_;
}

}  // namespace rgb_depth_sync
