# package to run instead of ur_driver for labs in ENME480

rostopic pub --once /ur3/command ur3e_mrc/command "{destination: [0, -1.57, -1.57, 0, 0, 0], v: 1.0, a: 1.0, io_0: false}"
