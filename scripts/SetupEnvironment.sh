export OpenCV_DIR=/home/ubuntu/opencv-3.1.0/build
v4l2-ctl -d /dev/video1 -c exposure_auto=1 -c exposure_absolute=1
v4l2-ctl -d /dev/video1 -c white_balance_temperature_auto=0
