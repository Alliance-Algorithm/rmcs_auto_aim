# rmcs_auto_aim

## Parameters

- ### exposure_time

Exposure time of the camera. The longer the exposure time, the brighter the image. Moreover, too brighter or too dark images will cause the model to fail.

- ### armor_predict_duration

The duration of the armor prediction. The longer the duration, the more delayed the marking of the armor target lost.

- ### buff_predict_duration

The duration of the buff prediction. The longer the duration, the more delayed the marking of the buff target lost.

- ### gimbal_predict_duration

The longer the duration, the more delayed the transmission of the zero vector.

- ### yaw_error

The error of the yaw angle to trajectory solution result.

- ### pitch_error

The error of the pitch angle to trajectory solution result.

- ### armor_model_path

The path of the armor model.

- ### buff_model_path

The path of the buff model.

- ### fx, fy, cx, cy

The intrinsic parameters of the camera.

- ### k1, k2, k3

The distortion parameters of the camera.

- ### omni_exposure

The exposure time of the omni-direction perception camera.

- ### omni_fx, omni_fy, omni_cx, omni_cy

The intrinsic parameters of the omni-direction perception camera.

- ### omni_k1, omni_k2, omni_k3

The distortion parameters of the omni-direction perception camera.

- ### record_fps

The fps of the recorded video.

- ### debug

Whether to debug.

- ### debug_color

The color of our own robot in the debug mode. 0 for blue, 1 for red.

- ### debug_robot_id

The robot id of our own robot in debug mode. 
- ID:
    - 0: Unknown
    - 1: Hero
    - 2: Engineer
    - 3: Infantry III
    - 4: Infantry IV
    - 5: Infantry V
    - 6: Aerial
    - 7: Sentry
    - 8: Dart
    - 9: Radar
    - 10: Outpost
    - 11: Base

- ### debug_buff_mode

Whether to debug the buff mode.

- ### record

Whether to record the video.

- ### raw_img_pub

Whether to publish the raw image.
