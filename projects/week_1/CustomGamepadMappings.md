Different gamepads map their inputs differently. By default, the launch file above uses a config that works with the joysticks we use in the classroom. You can create your own config file to set the mappings appropriate for your gamepad.

To show the content of the default config file, run the following command. You can then copy this to a file anywhere on your computer and edit it there.

```bash
$ cat $(ros2 pkg prefix traini_bringup)/share/traini_bringup/config/joystick_parameters.yaml
```

Then, you can launch the joystick control nodes with your new config like this:

```bash
$ ros2 launch traini_bringup joystick_control.launch.py config_path:=/path/to/your/config/file.yaml
```