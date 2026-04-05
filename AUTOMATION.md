- spawn controller processes with args passed in to specify the behaviour tree to use
  - one host (server) and a client over a http mock for the bluetooth channel

- simulator spawns and supplies recording id to controllers
- in interactive mode, everything connects to the same Rerun Viewer process

- simulator runs a http server with goals scored and other info

# headless operation

When running headlessly, a central server allocates jobs for which two behaviour trees to test,
clients spawn simulator instance and controller processes until termination, then report back to
central server storing in a DB for display from a web interface. `.rrd` files are merged from
the simulator and robot processes and uploaded alongside the leaderboard.
