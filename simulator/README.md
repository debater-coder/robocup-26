# RoboCup Simulator

Runs a simulator server. Robot controller processes connect to this server and call the
HTTP JSON API to send commands to control simulated robots and receive the state of the
virtual world. Simulated physics is 2D, and the simulated state is logged to Rerun.
