## Setting
1. Very important: set `NERVE_COMM = 0` and write firmware to spinal.

2. Connect like this:

 ``````````````````````````````````
 ``````````````````````````````````
 ```  PC --spinal--ESC---rotor  ```
 ```  |              |          ```
 ```  |-- power supply          ```
 ```  |                         ```
 ```  |--force sensor           ```
 ``````````````````````````````````
 ``````````````````````````````````
3. Create new wired setting for power supply. Set IP address of the power supply to `192.168.0.2`, Netmask to '255.255.255.0', and Gateway to 192.168.0.1.
4. Power on the power supply and set the voltage as you like (e.g. 25.2V).
5. In your terminal, do:
   ```
   $ roslaunch motor_test test.launch
   ```

   **parameter**:
   - `test_mode`: the test mode. Step mode (:=0) is to give continous steps command to the ESC, the problem is the bias of the force sensor increases during the steps commands. Thus, we recommand to use **one-shot** mode (:=1, default mode), which iteratively gives an increasing PWM value. The difference is this mode stops the motor rotation after each step, and re-calibrate the force sensor before the next step. So the bias of force sensor can be considered to be zero.
   - `run_duration`: the duration of motor rotation during each step.
   - `pwm_incremental_value`: the increamental value between two steps.
   - `min_pwm_value`/`max_pwm_value`: the min/max of the pwm value
   - `raise_duration`/`brake_duration`: these are the special parameters for one-shot mode, since we have to consider the raise-up phase after staring rotation and brake-down phase after stopping rotation. For small propellers (e.g. 5inch), these value can be small. For large propellers (e.g. 14inch), please increase there values
   - `force_sensor`: the type of force sensor. Options are CFS034CA301U (default); PFS055YA501U6 (larger type)
   - `motor_index`: -1 (default) commands all motors at once and records the ESC telemetry of motor 0. `0, 1, ...` command only the given motor and records its ESC telemetry.
   - `pwm_ramp_rate`: [pwm value / s] limits the rate of change of the command, also from and to `stop_pwm_value` when the motor starts and stops. The phases `raise`, `valid` and `brake` only count when a ramp is done, and in one-shot mode the log labels the rows of the ramps `ramp`. 0 (default) disables it, the command jumps.
   - `has_dshot_telemetry`: logs RPM, temperature and voltage from the ESC of the tested motor (default: false).
   - `calib_read_new_frames`: the force sensor calibration averages 1 s of new frames, instead of the latest frame repeated 50 times (default: false).

   **sample**:
   - for 5inch propeller:
     ```
     $ roslaunch motor_test test.launch max_pwm_value:=1900 raise_duration:=1.0 brake_duration:=2.0 run_duration:=2.0
     ```

   - for 5inch propeller:
     ```
     $ roslaunch motor_test test.launch max_pwm_value:=1200 raise_duration:=2  brake_duration:=5 run_duration:=2
     ```

6. Start logging by `rostopic pub -1 /start_log_cmd std_msgs/Empty "{}" `.

7. Cool down your motor by sending 'rostopic pub -1 /pwm_test spinal/PwmTest "{pwms: [0.54]}" ' and when it is cool enough, send 'rostopic pub -1 /pwm_test spinal/PwmTest "{pwms: [0.5]}" '，and then start a next test. Add e.g. 'motor_index: [1]' to address single motors only, and send an empty message 'rostopic pub -1 /pwm_test spinal/PwmTest "{}" ' to leave the pwm test mode.

## RPM mode (ESC with closed-loop RPM control)

If the ESC tracks a target RPM in closed loop (e.g. the AM32 fork with `drive_by_rpm`), the pwm value commands a target RPM:

```
pwm value [1000, 2000] -> duty [0.5, 1.0] -> DShot [48, 2047] -> target RPM [min RPM, max RPM] of the ESC
```

e.g. for an RPM window of 900 / 13300 RPM: target RPM ≈ 900 + 12.4 * (pwm value - 1000), and a `pwm_ramp_rate` of 100 /s ramps by 1240 RPM/s. Since duty 0.5 (`stop_pwm_value`) then holds the min RPM, the rotor would never stop between the steps. Use `rpm_mode:=true`, which:
- stops the motor with DShot 0: it sends an empty message to `/pwm_test`, upon which spinal leaves the pwm test mode. This assumes that no flight control has started the motors, as on the test bench. With `pwm_ramp_rate`, the command first ramps down to the min RPM.
- waits for the force sensor node and zeroes the force sensor right before the first step.
- adds the time stamp of each force measurement as the last column of the log.

Combine it with `has_dshot_telemetry:=true` (the measured RPM to identify the rotor with), `calib_read_new_frames:=true` and a `pwm_ramp_rate`. The rotor must be at rest when the test starts, i.e. spinal outside the pwm test mode.

**step by step**:
1. The RPM window of the ESC firmware (AM32 `MINIMUM_RPM_SPEED_CONTROL` / `MAXIMUM_RPM_SPEED_CONTROL`) defines the RPM of a pwm value, see above. The motor pole count of the ESC and of spinal (`num_motor_mag_pole_` in `dshot.cpp`) must both equal the magnet pole count of the motor.
2. Connect spinal (`/dev/ttyUSB0`) and the force sensor (`/dev/ttyACM0`) to the PC. Mount the rotor so that its axis is the z axis of the force sensor.
3. Terminal 1: `roslaunch spinal bridge.launch`.
4. Only if the force sensor is not `/dev/ttyACM0`: `rosparam set /cfs_device_name /dev/ttyACM1`.
5. Terminal 2:
   ```
   $ roslaunch motor_test test.launch force_sensor:=PFS055YA501U6 rpm_mode:=true has_dshot_telemetry:=true calib_read_new_frames:=true motor_index:=2 pwm_ramp_rate:=200 min_pwm_value:=1000 max_pwm_value:=1900 pwm_incremental_value:=50 raise_duration:=1.0 run_duration:=2.0 brake_duration:=4.0
   ```
   The output shows the parameters, `Averaged ... new frames` and `done force sensor init calib`. Without a Takasago power supply, `power_node` fails, which does not matter (the column `currency` stays 0). `rostopic hz /cfs/data` should show about 100 Hz.
6. Optional, terminal 3: `rosbag record -a`.
7. Terminal 3: `rostopic pub -1 /start_log_cmd std_msgs/Empty "{}"`. Each step ramps up, settles (`raise`), measures (`valid`), ramps down, stops, coasts (`brake`) and zeroes the force sensor. The node prints `finish pwm test` at the end.
8. To abort: stopping `motor_test` does **not** stop the motor, spinal holds the last command. Send `rostopic pub -1 /pwm_test spinal/PwmTest "{}"` (DShot 0 right away), or cut the power.
9. The log is in `~/.ros/`.

**analysis**: identify `f_t = k_t * Omega^2` and `tau_t = k_tau * Omega^2` with `Omega` the measured rotor speed in rad/s:
```
$ python3 scripts/analyze_data.py <YYYYMMDD>_motor_test_motor<motorid>.txt 1 --rpm_mode 1
```
Add `--if_reverse 1` if the thrust points along -z of the force sensor (the script warns then).

Each step is the mean of its `valid` rows minus the zero offset of the force sensor at that time: the calibration zeroes the sensor right before the step, the last `--zero_samples` (default: 50) `brake` rows of the step at rest (RPM 0) measure the offset again, and in between the offset is interpolated linearly in time to remove the drift. Both fits go through the origin. The script also prints the rows per second: the force sensor node aims at 100 Hz, but publishes fewer frames if the sensor sends at about 1 or 2 kHz. The script also prints `k_t` in N/kRPM^2 (`krpm_square_to_thrust_ratio` with `krpm_square_to_thrust_bias: 0`) and `k_tau / k_t` (`m_f_rate`).

## General calib for ESC:

Low PWM: 1000 (start with 1050)
High PWM: 1900