# What are the current camera warnings?
```txt
After adding failure recovery (BEST_EFFORT QoS, ThreadLock, Circuit Breaker)
```
The current warnings are:

- `Camera calibration file not found`
- `unknown control 'white_balance_temperature_auto'`
- `unknown control 'exposure_auto'`
- `unknown control 'focus_auto'`

### A. Camera calibration file not found

Log:

```txt
Camera calibration file /home/dacekey/.ros/camera_info/default_cam.yaml not found
```

Meaning:

usb_cam is looking for the calibration file for the camera named default_cam, but that file does not exist yet.

This is not a serious issue at the moment because we have not performed real camera calibration yet.

This only matters if we want the published camera_info topic to contain real camera intrinsics and distortion coefficients.

Currently, the 3D Localizer is using its own configuration:

`kfs_3d_localizer.camera_matrix.*`
`kfs_3d_localizer.distortion.*`

Therefore, the missing default_cam.yaml file does not crash the node.

Later, after running camera calibration, there are two possible directions:

Save the calibration result to `~/.ros/camera_info/default_cam.yaml` for `camera_info`.
Copy `fx/fy/cx/cy` and distortion coefficients into `yolo_detection.yaml` for `kfs_3d_localizer`.

### B. unknown control 'white_balance_temperature_auto'

Meaning:

usb_cam tries to set the control `white_balance_temperature_auto`, but the current camera does not support this control.

This is not a critical error. It only means that the device `/dev/video0` does not expose this control.

### C. unknown control 'exposure_auto'

Similarly, the camera does not support the control named `exposure_auto`.

This may happen because the camera driver uses a different control name, for example:

`auto_exposure`
`exposure_time_absolute`
`exposure_dynamic_framerate`

The exact control names depend on the camera and driver.

### D. unknown control 'focus_auto'

Meaning:

The camera does not support auto focus, or the driver does not expose `focus_auto`.

For many fixed-focus camera modules or simple webcams, this is normal.

## 3. Do we need to fix the camera warnings immediately?

Not yet, as long as the camera is still running correctly.

The following logs show that the camera is still publishing frames:

```txt
Starting 'default_cam' (/dev/video0) at 640x480 via mmap (yuyv) at 30 FPS
Timer triggering every 33 ms
```

However, there is one important point to notice:

```txt
Starting at 640x480 via yuyv at 30 FPS
```

Meanwhile, the previous 3D Localizer configuration is still using placeholder values:

```txt
image_width: 1280
image_height: 720
```

If the real runtime resolution is `640x480`, then the 3D Localizer configuration must match it later:

`kfs_3d_localizer.image.width: 640`
`kfs_3d_localizer.image.height: 480`

Alternatively, we can change the camera runtime mode to `1280x720 MJPEG`.

For the current flow test, this is acceptable. However, when performing real 3D localization, the runtime resolution must match the calibration resolution and the 3D Localizer configuration.
