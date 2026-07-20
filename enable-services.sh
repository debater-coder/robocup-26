systemctl enable ./robocup-camera.service
systemctl enable ./robocup-controller.service
sudo systemctl daemon-reload
sudo systemctl start robocup-camera
sudo systemctl start robocup-controller
