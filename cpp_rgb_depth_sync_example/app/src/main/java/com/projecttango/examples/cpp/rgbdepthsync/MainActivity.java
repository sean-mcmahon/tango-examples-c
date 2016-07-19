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

package com.projecttango.examples.cpp.rgbdepthsync;

import android.app.Activity;
import android.content.ComponentName;
import android.content.ServiceConnection;
import android.graphics.Point;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.os.IBinder;
import android.util.Log;
import android.view.Display;
import android.view.View;
import android.view.WindowManager;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;
import android.os.Handler;
import android.os.Handler.Callback;
import android.os.Message;

import com.projecttango.examples.cpp.util.TangoInitializationHelper;

/**
 * Activity that load up the main screen of the app, this is the launcher activity.
 */
public class MainActivity extends Activity {
  private static final String TAG = MainActivity.class.getSimpleName();

  // The minimum Tango Core version required from this application.
  private static final int MIN_TANGO_CORE_VERSION = 9377;
  long starttime = 0;

  private GLSurfaceRenderer mRenderer;
  private GLSurfaceView mGLView;

  private SeekBar mDepthOverlaySeekbar;
  private CheckBox mdebugOverlayCheckbox;
  private CheckBox mGPUUpsampleCheckbox;
  private CheckBox mDataRecordingCheckbox;

  // Timer initialisation stuff
  TextView mtext;
//  final Handler h = new Handler(new Callback() {
//
//    @Override
//    public boolean handleMessage(Message msg) {
//      long millis = System.currentTimeMillis() - starttime;
//      int seconds = (int) (millis / 1000);
//      int minutes = seconds / 60;
//      seconds     = seconds % 60;
//
//      mtext.setText(String.format("%d:%02d", minutes, seconds));
//      return false;
//    }
//  });
  Handler h2 = new Handler();
  Runnable run = new Runnable() {

    @Override
    public void run() {
      long millis = System.currentTimeMillis() - starttime;
      int seconds = (int) (millis / 1000);
      int minutes = seconds / 60;
      seconds     = seconds % 60;

      mtext.setText(String.format("%d:%02d", minutes, seconds));

      if (seconds >=2) {
        // Cancel recording
        mDataRecordingCheckbox.setChecked(false);
      }

      h2.postDelayed(this, 500);
    }
  };

  // Tango Service connection.
  ServiceConnection mTangoServiceConnection = new ServiceConnection() {
      public void onServiceConnected(ComponentName name, IBinder service) {
        JNIInterface.onTangoServiceConnected(service);

        if (!JNIInterface.tangoSetIntrinsicsAndExtrinsics()) {
          Log.e(TAG, "Failed to set extrinsics and intrinsics.");
          finish();
        }

        if (!JNIInterface.tangoSetupConfig()) {
          Log.e(TAG, "Failed to set config.");
          finish();
        }

        if (!JNIInterface.tangoConnectCallbacks()) {
          Log.e(TAG, "Failed to set connect callbacks.");
          finish();
        }

        if (!JNIInterface.tangoConnect()) {
          Log.e(TAG, "Failed to set connect service.");
          finish();
        }
      }

      public void onServiceDisconnected(ComponentName name) {
        // Handle this if you need to gracefully shutdown/retry
        // in the event that Tango itself crashes/gets upgraded while running.
      }
    };

  private class DepthOverlaySeekbarListener implements SeekBar.OnSeekBarChangeListener {
    @Override
    public void onProgressChanged(SeekBar seekBar, int progress,
                                  boolean fromUser) {
      JNIInterface.setDepthAlphaValue((float) progress / (float) seekBar.getMax());
    }

    @Override
    public void onStartTrackingTouch(SeekBar seekBar) {}

    @Override
    public void onStopTrackingTouch(SeekBar seekBar) {}
  }

  private class DebugOverlayCheckboxListener implements CheckBox.OnCheckedChangeListener {
    @Override
    public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
      if (buttonView == mdebugOverlayCheckbox) {
        if (isChecked) {
          float progress = mDepthOverlaySeekbar.getProgress();
          float max = mDepthOverlaySeekbar.getMax();
          JNIInterface.setDepthAlphaValue(progress / max);
          mDepthOverlaySeekbar.setVisibility(View.VISIBLE);
        } else {
          JNIInterface.setDepthAlphaValue(0.0f);
          mDepthOverlaySeekbar.setVisibility(View.GONE);
        }
      }
    }
  }

  private class GPUUpsampleListener implements CheckBox.OnCheckedChangeListener {
    @Override
    public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
      JNIInterface.setGPUUpsample(isChecked);
    }
  }

  private class DataRecordingListener implements CheckBox.OnCheckedChangeListener {
    @Override
    public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
      JNIInterface.SetDataRecording(isChecked);
      if (isChecked) {
        // start runable
        starttime = System.currentTimeMillis();
        h2.postDelayed(run, 0);
      }
      else {
        starttime = 0;
        h2.removeCallbacks(run);
        mtext.setText(String.format("Not Recording"));
      }
    }
  }

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);

    Display display = getWindowManager().getDefaultDisplay();
    Point size = new Point();
    display.getSize(size);

    getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
            WindowManager.LayoutParams.FLAG_FULLSCREEN);

    setContentView(R.layout.activity_main);

    // Set my timer stuff
    mtext = (TextView)findViewById(R.id.data_recording_textview);

    mDepthOverlaySeekbar = (SeekBar) findViewById(R.id.depth_overlay_alpha_seekbar);
    mDepthOverlaySeekbar.setOnSeekBarChangeListener(new DepthOverlaySeekbarListener());
    mDepthOverlaySeekbar.setVisibility(View.GONE);

    mdebugOverlayCheckbox = (CheckBox) findViewById(R.id.debug_overlay_checkbox);
    mdebugOverlayCheckbox.setOnCheckedChangeListener(new DebugOverlayCheckboxListener());

    mGPUUpsampleCheckbox = (CheckBox) findViewById(R.id.gpu_upsample_checkbox);
    mGPUUpsampleCheckbox.setOnCheckedChangeListener(new GPUUpsampleListener());

    mDataRecordingCheckbox = (CheckBox) findViewById(R.id.data_recording_checkbox);
    mDataRecordingCheckbox.setOnCheckedChangeListener( new DataRecordingListener());

    // OpenGL view where all of the graphics are drawn
    mGLView = (GLSurfaceView) findViewById(R.id.gl_surface_view);

    // Configure OpenGL renderer
    mGLView.setEGLContextClientVersion(2);
    mRenderer = new GLSurfaceRenderer(this);
    mGLView.setRenderer(mRenderer);

    // Check that the installed version of the Tango Core is up to date.
    if (!JNIInterface.checkTangoVersion(this, MIN_TANGO_CORE_VERSION)) {
      Toast.makeText(this, "Tango Core out of date, please update in Play Store",
                     Toast.LENGTH_LONG).show();
      finish();
      return;
    }
  }

  @Override
  protected void onResume() {
    // We moved most of the onResume lifecycle calls to the surfaceCreated,
    // surfaceCreated will be called after the GLSurface is created.
    super.onResume();
    mGLView.onResume();
    TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);
  }

  @Override
  protected void onPause() {
    super.onPause();
    mGLView.onPause();
    JNIInterface.tangoDisconnect();
    unbindService(mTangoServiceConnection);
    h2.removeCallbacks(run);
  }

  public void surfaceCreated() {
    // This gets called in GLsurfaceRenderer.java an instance of this class is passed there and this method is called
    JNIInterface.initializeGLContent();
    if (!JNIInterface.tangoConnectTexture()) {
      Log.e(TAG, "Failed to connect texture.");
      finish();
    }
  }

  /*public void FisheyeSurfaceCreated() {
    // This gets called in GLsurfaceRenderer.java an instance of this class is passed there and this method is called
    if (!JNIInterface.tangoConnectFisheyeTexture()) {
      Log.e(TAG, "Failed to connect texture.");
      finish();
    }
  } */
}
