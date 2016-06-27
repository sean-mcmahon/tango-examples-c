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

void SynchronizationApplication::OnXYZijAvailable(const TangoXYZij* xyz_ij) {
  // We'll just update the point cloud associated with our depth image.
  TangoSupport_updatePointCloud(point_cloud_manager_, xyz_ij);
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
  if (err != TANGO_SUCCESS) {
    LOGE("Failed to enable depth.");
    return false;
  }

  // We also need to enable the color camera in order to get RGB frame
  // callbacks.
  err = TangoConfig_setBool(tango_config_, "config_enable_color_camera", true);
  if (err != TANGO_SUCCESS) {
    LOGE(
        "Failed to set 'enable_color_camera' configuration flag with error"
        " code: %d",
        err);
    return false;
  }

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
        TangoCameraIntrinsics color_camera_intrinsics;
        TangoErrorType err = TangoService_getCameraIntrinsics(
                TANGO_CAMERA_COLOR, &color_camera_intrinsics);
        if (err != TANGO_SUCCESS) {
            LOGE(
                    "SynchronizationApplication: Failed to get the intrinsics for the color"
                            "camera for intialising image buffer manager.");
            return false;
        }
        int image_width_ = color_camera_intrinsics.width;
        int image_height_ = color_camera_intrinsics.height;
        err = TangoSupport_createImageBufferManager(TANGO_HAL_PIXEL_FORMAT_RGBA_8888, image_width_,
                                                    image_height_, &color_image_manager_);
        if (err != TANGO_SUCCESS)
        {
            LOGE("SynchronizationApplication: Failed to create image buffer manager.");
            return false;
        }
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

bool SynchronizationApplication::tangoConnectFisheyeTexture() {
    // The Tango service allows you to connect an OpenGL texture directly to its
    // RGB and fisheye cameras. This is the most efficient way of receiving
    // images from the service because it avoids copies. You get access to the
    // graphic buffer directly. As we're interested in rendering the color image
    // in our render loop, we'll be polling for the color image as needed.
    TangoErrorType err = TangoService_connectTextureId(
            TANGO_CAMERA_FISHEYE, fisheye_image_.GetTextureId(), this, nullptr);
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
  return depth_ret == TANGO_SUCCESS;
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
  depth_image_.SetCameraIntrinsics(color_camera_intrinsics);
  main_scene_.SetCameraIntrinsics(color_camera_intrinsics);

  // Initialize TangoSupport context.
  TangoSupport_initialize(TangoService_getPoseAtTime);
  return true;
}

void SynchronizationApplication::TangoDisconnect() {
  TangoService_disconnect();
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
  bool new_points = false;
  bool new_pointsTwo = false;
    double timediff = 1.0;
    double difference = 0.0;
  TangoSupport_getLatestPointCloudAndNewDataFlag(point_cloud_manager_,
                                                 &render_buffer_, &new_points);
  depth_timestamp = render_buffer_->timestamp;
  // We need to make sure that we update the texture associated with the color
  // image.
  if (TangoService_updateTexture(TANGO_CAMERA_COLOR, &color_timestamp) !=
      TANGO_SUCCESS) {
    LOGE("SynchronizationApplication: Failed to get a color image.");
  }
    // Get latest colour image manger and buffer.
  if (TangoSupport_getLatestImageBufferAndNewDataFlag(color_image_manager_, &color_image_buffer_,&new_pointsTwo) != TANGO_SUCCESS) {
        LOGE("SynchronizationApplication: Failed to get latest color image buffer.");
    }

  // In the following code, we define t0 as the depth timestamp and t1 as the
  // color camera timestamp.

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
                                          new_points);
      } else {
        depth_image_.UpdateAndUpsampleDepth(color_image_t1_T_depth_image_t0,
                                            render_buffer_);
      }
    // Save the colour image (color_image_buffer_ (struct)), depth image
    // color_image_buffer_.depth_map_buffer_ (float vector), point cloud render_buffer_
    // and pose info pose_color_image_t1_T_depth_image_t0.
    // Save ONE instance of TangoCameraIntrinsics per recording
    // Save any instance of TangoEvent
    TangoEvent* myEventInstance;

    // this isn't technically every second, as depth_timestamp only updates when a new point cloud
    // has been recorded.
    difference = depth_timestamp - time_buffer_;
    if (difference >= timediff)
    {
        // Save stuff
        LOGI("Saving Data at Timestamp: %f with time buffer %f ", depth_timestamp, time_buffer_);


        time_buffer_ = depth_timestamp;

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

}  // namespace rgb_depth_sync
