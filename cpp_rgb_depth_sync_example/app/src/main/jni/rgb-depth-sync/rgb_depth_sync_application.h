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

#ifndef RGB_DEPTH_SYNC_RGB_DEPTH_SYNC_APPLICATION_H_
#define RGB_DEPTH_SYNC_RGB_DEPTH_SYNC_APPLICATION_H_

#include <jni.h>
#include <list>
#include <iterator>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string.h>
#include <stdlib.h>

#include <tango_client_api.h>
#include <tango_support_api.h>
#include <rgb-depth-sync/color_image.h>
#include <rgb-depth-sync/depth_image.h>
#include <rgb-depth-sync/fisheye_image.h>
#include <rgb-depth-sync/scene.h>
#include <rgb-depth-sync/util.h>
#include <tango-gl/util.h>

namespace rgb_depth_sync {

// This thread safe class is the main application for Synchronization.
// It can be instantiated in the JNI layer and use to pass information back and
// forth between Java. The class also manages the application's lifecycle and
// interaction with the Tango service. Primarily, this involves registering for
// callbacks and passing on the necessary information to stored objects. It also
// takes care of passing a vector container which has a pointer to the
// latest point cloud buffer that is to used for rendering.
//  To reduce the number of point cloud data copies between callback and render
// threads we use three buffers which are synchronously exchanged between each
// other so that the render loop always contains the latest point cloud data.
// 1. Callback buffer : The buffer to which pointcloud data received from Tango
// Service callback is copied out.
// 2. Shared buffer: This buffer is used to share the data between Service
// callback and Render loop
// 3. Render Buffer: This buffer is used in the renderloop to project point
// cloud data to a 2D image plane which is of the same size as RGB image. We
// also make sure that this buffer contains the latest point cloud data that is
//  received from the call back.
class SynchronizationApplication {
 public:
  SynchronizationApplication();
  ~SynchronizationApplication();

  // Check that the installed version of the Tango API is up to date
  // and initialize other data.
  //
  // @return returns true if the application version is compatible with the
  //         Tango Core version.
  bool CheckTangoVersion(JNIEnv* env, jobject activity, int min_tango_version);

  // Called when Tango Service is connected successfully.
  void OnTangoServiceConnected(JNIEnv* env, jobject binder);

  // Setup the configuration file for the Tango Service. .
  bool TangoSetupConfig();

  // Associate the texture generated from an Opengl context to which the color
  // image will be updated to.
  bool TangoConnectTexture();

  // Sets the callbacks for OnXYZijAvailable
  bool TangoConnectCallbacks();

  // Connect to Tango Service.
  // This function will start the Tango Service pipeline, in this case, it will
  // start Depth Sensing callbacks.
  bool TangoConnect();

  // Queries and sets the camera transforms between different sensors of
  // Project Tango Device that are required to project Point cloud onto
  // Image plane.
  bool TangoSetIntrinsicsAndExtrinsics();

  // Disconnect from Tango Service.
  void TangoDisconnect();

  // Inititalizes all the OpenGL resources required to render a Depth Image on
  // Top of an RGB image.
  void InitializeGLContent();

  // Setup the view port width and height.
  void SetViewPort(int width, int height);

  // Main Render loop.
  void Render();

  // SecondaryFisheye Render loop.
 // void RenderFisheye();

  // Set the transparency of Depth Image.
  void SetDepthAlphaValue(float alpha);

  // Set whether to use GPU or CPU upsampling
  void SetGPUUpsample(bool on);

  // Callback for point clouds that come in from the Tango service.
  //
  // @param xyz_ij The point cloud returned by the service.
  //
  void OnXYZijAvailable(const TangoXYZij* xyz_ij);

  // Color bullshit
  void OnFrameAvailable(const TangoImageBuffer* buffer);

 private:

 void writeCameraIntrinsics2Text(const TangoCameraIntrinsics tango_camera_intrinsics_);

 TangoImageBuffer getImageClosestToTS(std::list<TangoImageBuffer> image_list_, const double depth_timestamp);

  // RGB image
  ColorImage color_image_;

  // RGB image
  FisheyeImage fisheye_image_;

  // Depth image created by projecting Point Cloud onto RGB image plane.
  DepthImage depth_image_;

  // Main scene which contains all the renderable objects.
  Scene main_scene_;

  // Tango configration file, this object is for configuring Tango Service setup
  // before connect to service. For example, we turn on the depth sensing in
  // this example.
  TangoConfig tango_config_;

  // OpenGL to Start of Service
  glm::mat4 OW_T_SS_;
  float screen_width_;
  float screen_height_;

  // The point_cloud_manager allows for thread safe reading and
  // writing of the point cloud data.
  TangoSupportPointCloudManager* point_cloud_manager_;

 // ?? The image buffer manager allows for thread safe reading and writing of the
 // color image data
  TangoSupportImageBufferManager* color_image_manager_;

  // This TangoXYZij* points to the most recently produced
  // point cloud data which should be rendered.
  TangoXYZij* render_buffer_;
  TangoImageBuffer* color_image_buffer_;
  std::list<TangoImageBuffer> color_buffer_list_;
  std::list<TangoImageBuffer>::iterator image_list_iterator_;

 // Image Sizes
 int image_width_;
 int image_height_;
 const int image_depth_ = 3;

 // Used to save data every x seconds
 double time_buffer_ = 0.0;
 bool saving_to_file_ = false;
 bool successful_color_image_retreval = false;

 std::ofstream myfile;

 unsigned int num_write_iterations = 0;
 //char my_file_name_[] = "2ts_all_data.bin";

  bool gpu_upsample_;
};
}  // namespace rgb_depth_sync

#endif  // RGB_DEPTH_SYNC_RGB_DEPTH_SYNC_APPLICATION_H_
