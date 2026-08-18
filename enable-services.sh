systemctl enable ./robocup-camera.service
systemctl enable ./robocup-controller.service
systemctl enable ./robocup-ui.service
sudo systemctl daemon-reload
sudo systemctl start robocup-ui
sudo systemctl start robocup-camera
sudo systemctl start robocup-controller
