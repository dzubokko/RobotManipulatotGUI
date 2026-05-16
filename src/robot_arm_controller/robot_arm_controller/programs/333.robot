{
  "name": "333",
  "format": "robot_dsl_v2",
  "description": "",
  "created_or_updated_at": "2026-05-17T00:07:08.731911",
  "source": "reset_home()\nwait(1.0)\n\ngrip_open()\nwait(0.5)\ngrip_close()\nwait(0.5)\n\nmove_lin(0.005, 0.0, 0.0, 1.0)\nwait(0.5)\nmove_lin(-0.005, 0.0, 0.0, 1.0)\nwait(0.5)\n\nmove_lin(0.0, 0.005, 0.0, 1.0)\nwait(0.5)\nmove_lin(0.0, -0.005, 0.0, 1.0)\nwait(0.5)\n\nmove_lin(0.0, 0.0, 0.005, 1.0)\nwait(0.5)\nmove_lin(0.0, 0.0, -0.005, 1.0)\nwait(0.5)\n\nrotate_rx(5.0, 1.0)\nwait(0.5)\nrotate_rx(-5.0, 1.0)\nwait(0.5)\n\nrotate_ry(5.0, 1.0)\nwait(0.5)\nrotate_ry(-5.0, 1.0)\nwait(0.5)\n\nrotate_rz(5.0, 1.0)\nwait(0.5)\nrotate_rz(-5.0, 1.0)\nwait(0.5)\n\nreset_home()",
  "commands": [
    {
      "line_no": 1,
      "name": "reset_home",
      "args": [],
      "raw": "reset_home()"
    },
    {
      "line_no": 2,
      "name": "wait",
      "args": [
        1.0
      ],
      "raw": "wait(1.0)"
    },
    {
      "line_no": 4,
      "name": "grip_open",
      "args": [],
      "raw": "grip_open()"
    },
    {
      "line_no": 5,
      "name": "wait",
      "args": [
        0.5
      ],
      "raw": "wait(0.5)"
    },
    {
      "line_no": 6,
      "name": "grip_close",
      "args": [],
      "raw": "grip_close()"
    },
    {
      "line_no": 7,
      "name": "wait",
      "args": [
        0.5
      ],
      "raw": "wait(0.5)"
    },
    {
      "line_no": 9,
      "name": "move_lin",
      "args": [
        0.005,
        0.0,
        0.0,
        1.0
      ],
      "raw": "move_lin(0.005, 0.0, 0.0, 1.0)"
    },
    {
      "line_no": 10,
      "name": "wait",
      "args": [
        0.5
      ],
      "raw": "wait(0.5)"
    },
    {
      "line_no": 11,
      "name": "move_lin",
      "args": [
        -0.005,
        0.0,
        0.0,
        1.0
      ],
      "raw": "move_lin(-0.005, 0.0, 0.0, 1.0)"
    },
    {
      "line_no": 12,
      "name": "wait",
      "args": [
        0.5
      ],
      "raw": "wait(0.5)"
    },
    {
      "line_no": 14,
      "name": "move_lin",
      "args": [
        0.0,
        0.005,
        0.0,
        1.0
      ],
      "raw": "move_lin(0.0, 0.005, 0.0, 1.0)"
    },
    {
      "line_no": 15,
      "name": "wait",
      "args": [
        0.5
      ],
      "raw": "wait(0.5)"
    },
    {
      "line_no": 16,
      "name": "move_lin",
      "args": [
        0.0,
        -0.005,
        0.0,
        1.0
      ],
      "raw": "move_lin(0.0, -0.005, 0.0, 1.0)"
    },
    {
      "line_no": 17,
      "name": "wait",
      "args": [
        0.5
      ],
      "raw": "wait(0.5)"
    },
    {
      "line_no": 19,
      "name": "move_lin",
      "args": [
        0.0,
        0.0,
        0.005,
        1.0
      ],
      "raw": "move_lin(0.0, 0.0, 0.005, 1.0)"
    },
    {
      "line_no": 20,
      "name": "wait",
      "args": [
        0.5
      ],
      "raw": "wait(0.5)"
    },
    {
      "line_no": 21,
      "name": "move_lin",
      "args": [
        0.0,
        0.0,
        -0.005,
        1.0
      ],
      "raw": "move_lin(0.0, 0.0, -0.005, 1.0)"
    },
    {
      "line_no": 22,
      "name": "wait",
      "args": [
        0.5
      ],
      "raw": "wait(0.5)"
    },
    {
      "line_no": 24,
      "name": "rotate_rx",
      "args": [
        5.0,
        1.0
      ],
      "raw": "rotate_rx(5.0, 1.0)"
    },
    {
      "line_no": 25,
      "name": "wait",
      "args": [
        0.5
      ],
      "raw": "wait(0.5)"
    },
    {
      "line_no": 26,
      "name": "rotate_rx",
      "args": [
        -5.0,
        1.0
      ],
      "raw": "rotate_rx(-5.0, 1.0)"
    },
    {
      "line_no": 27,
      "name": "wait",
      "args": [
        0.5
      ],
      "raw": "wait(0.5)"
    },
    {
      "line_no": 29,
      "name": "rotate_ry",
      "args": [
        5.0,
        1.0
      ],
      "raw": "rotate_ry(5.0, 1.0)"
    },
    {
      "line_no": 30,
      "name": "wait",
      "args": [
        0.5
      ],
      "raw": "wait(0.5)"
    },
    {
      "line_no": 31,
      "name": "rotate_ry",
      "args": [
        -5.0,
        1.0
      ],
      "raw": "rotate_ry(-5.0, 1.0)"
    },
    {
      "line_no": 32,
      "name": "wait",
      "args": [
        0.5
      ],
      "raw": "wait(0.5)"
    },
    {
      "line_no": 34,
      "name": "rotate_rz",
      "args": [
        5.0,
        1.0
      ],
      "raw": "rotate_rz(5.0, 1.0)"
    },
    {
      "line_no": 35,
      "name": "wait",
      "args": [
        0.5
      ],
      "raw": "wait(0.5)"
    },
    {
      "line_no": 36,
      "name": "rotate_rz",
      "args": [
        -5.0,
        1.0
      ],
      "raw": "rotate_rz(-5.0, 1.0)"
    },
    {
      "line_no": 37,
      "name": "wait",
      "args": [
        0.5
      ],
      "raw": "wait(0.5)"
    },
    {
      "line_no": 39,
      "name": "reset_home",
      "args": [],
      "raw": "reset_home()"
    }
  ]
}