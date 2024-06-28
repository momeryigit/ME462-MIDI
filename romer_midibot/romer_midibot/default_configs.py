{
    "SENSORS": {
        "imu": {
            "id": 1,
            "enabled": False,
            "scl_pin": 1,
            "sda_pin": 0,
            "polling_rate": 50
        },
        "bumper_switches": {
            "polling_rate": 1,
            "sensors": [
                {"id": 1, "enabled": True, "pin": 10},
                {"id": 2, "enabled": True, "pin": 11},
                {"id": 3, "enabled": True, "pin": 12},
                {"id": 4, "enabled": True, "pin": 13}
            ]
        },
        "ultrasonic": {
            "polling_rate": 2,
            "sensors": [
                {"id": 1, "enabled": True, "trigger_pin": 22, "echo_pin": 26},
                {"id": 2, "enabled": True, "trigger_pin": 16, "echo_pin": 17},
                {"id": 3, "enabled": True, "trigger_pin": 18, "echo_pin": 19},
                {"id": 4, "enabled": True, "trigger_pin": 27, "echo_pin": 28}
            ]
        }
    },
    "stepper_motors": [
        {
            "id": 2,
            "enabled": True,
            "step_pin": 3,
            "dir_pin": 4,
            "enable_pin": 2,
            "np_id": 2,
            "np_start" : 0,
            "np_end" : 6,
            "acc_step_size": 50,
            "acc_timer_period": 10,
            "invert_dir": False
        },
        {
            "id": 1,
            "enabled": True,
            "step_pin": 6,
            "dir_pin": 7,
            "enable_pin": 5,
            "np_id": 1,
            "np_start" : 0,
            "np_end" : 6,
            "acc_step_size": 50,
            "acc_timer_period": 10,
            "invert_dir": True
        }
    ],
    "default_emergency_behavior": True,
    "heartbeat": {
        "enabled": False,
        "timer": 10000
    },
    "neopixel_strips":[
        {
            "id": 1,
            "enabled": True,
            "pin": 8,
            "num_pixels": 10
        },
        {
            "id": 2,
            "enabled": True,
            "pin": 9,
            "num_pixels": 10
        }
    ],
    "robot_parameters": {
        "microstepping": 0.5,
        "wheel_radius": 0.045,
        "wheel_separation": 0.295,
        "ticks_per_rev_full_step": 0.5
    }
}


