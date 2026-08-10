{
  "name": "Example motion",
  "format": "robot_dsl_v2",
  "description": "Basic motion sequence demonstrating gripper, Cartesian translation and TCP rotation.",
  "source": "reset_home()\nwait(1.0)\n\ngrip_open()\nwait(0.5)\ngrip_close()\nwait(0.5)\n\nmove_lin(0.005, 0.0, 0.0, 1.0)\nmove_lin(-0.005, 0.0, 0.0, 1.0)\n\nmove_lin(0.0, 0.005, 0.0, 1.0)\nmove_lin(0.0, -0.005, 0.0, 1.0)\n\nmove_lin(0.0, 0.0, 0.005, 1.0)\nmove_lin(0.0, 0.0, -0.005, 1.0)\n\nrotate_rx(5.0, 1.0)\nrotate_rx(-5.0, 1.0)\nrotate_ry(5.0, 1.0)\nrotate_ry(-5.0, 1.0)\nrotate_rz(5.0, 1.0)\nrotate_rz(-5.0, 1.0)\n\nreset_home()",
  "commands": []
}
