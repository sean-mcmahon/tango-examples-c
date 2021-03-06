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

#include "rgb-depth-sync/fisheye_image.h"

namespace rgb_depth_sync {

FisheyeImage::FisheyeImage() : texture_id_(0) {}

void FisheyeImage::InitializeGL() {
  // Generates the texture from the GLuint texture id
  glGenTextures(1, &texture_id_);
  // Specifies the kind of texture (YUV colour space)
  glBindTexture(GL_TEXTURE_EXTERNAL_OES, texture_id_);
  // Interpolates the image for different scren sizes
  glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  // Not sure about this, I think it's cleaning up
  glBindTexture(GL_TEXTURE_EXTERNAL_OES, 0);
}

    FisheyeImage::~FisheyeImage() {}
}  // namespace rgb_depth_sync
