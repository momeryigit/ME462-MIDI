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
            "polling_rate": 10,
            "sensors": [
                {"id": 1, "enabled": False, "pin": 10},
                {"id": 2, "enabled": False, "pin": 11},
                {"id": 3, "enabled": False, "pin": 12},
                {"id": 4, "enabled": False, "pin": 13}
            ]
        },
        "ultrasonic": {
            "polling_rate": 20,
            "sensors": [
                {"id": 1, "enabled": False, "trigger_pin": 14, "echo_pin": 15},
                {"id": 2, "enabled": False, "trigger_pin": 16, "echo_pin": 17},
                {"id": 3, "enabled": False, "trigger_pin": 18, "echo_pin": 19},
                {"id": 4, "enabled": False, "trigger_pin": 20, "echo_pin": 21}
            ]
        }
    },
    "stepper_motors": [
        {
            "id": 1,
            "enabled": False,
            "step_pin": 3,
            "dir_pin": 4,
            "enable_pin": 2,
            "led_pin": 8,
            "step_size": 50,
            "acc_timer_period": 10
        },
        {
            "id": 2,
            "enabled": False,
            "step_pin": 6,
            "dir_pin": 7,
            "enable_pin": 5,
            "led_pin": 9,
            "step_size": 50,
            "acc_timer_period": 10
        }
    ],
    "default_emergency_behavior": False,
    "heartbeat": {
        "enabled": False,
        "timer": 10000
    }
}
