# SPOT Troubleshooting

This page is for teams using the **Java** versions of SPOT (`mecanum/` and `starterbot/`). Blocks teams should use the Troubleshooting section of the [SPOT for Blocks guide](blocks/SPOT-Blocks-Guide.html#troubleshooting), which covers the same problems in Blocks terms.

Almost every SPOT problem is a sensor or a motor set up backwards. Work through the checks in this order:

1. **Pinpoint Test** (`PinpointTest.java`). Push the robot by hand and compare the readings to [the table below](#what-the-pinpoint-test-should-show). This finds pod and Pinpoint problems.
2. **Motor Direction Test** (`MotorDirectionTest.java`). Run it only once the Pinpoint Test passes. It finds reversed or swapped motors.
3. **Tuning.** Tune only once both tests pass and the robot is just a little off.

> **The Java versions have no runaway stop.** If a pod or motor is reversed, `drive()`, `strafe()` or `turnTo()` will keep the robot going until you press Stop. Run every new autonomous at power 0.3 the first time, with a thumb on Stop. (The Blocks version, `SpotBlocks`, stops the robot itself and shows a message.)

## Where each setting lives

| Setting | Mecanum | StarterBot |
|---|---|---|
| Motor directions | The four `setupDriveMotor(...)` / `setupMotor(...)` lines in **all four** files: `SimplifiedOdometryRobot.java`, `MotorDirectionTest.java`, `PinpointTest.java` and `FieldCentricTeleOp.java` | `setupDriveMotor(...)` calls in `initialize()` |
| Pod directions | `setEncoderDirections(...)` in `SimplifiedOdometryRobot.java`, `PinpointTest.java` **and** `FieldCentricTeleOp.java` | `setEncoderDirections(...)` in `initialize()` |
| Pod type | `setEncoderResolution(...)`, same three files | `setEncoderResolution(...)` |
| Pod offsets | `POD_OFFSET_X_MM` / `POD_OFFSET_Y_MM`, same three files | `POD_OFFSET_X_MM` / `POD_OFFSET_Y_MM` |

**The mecanum files each set up their own motors and Pinpoint.** If you fix a pod direction in `PinpointTest.java` but not in `SimplifiedOdometryRobot.java`, the test will pass and the autonomous will still run away. Make every change in every file listed.

## What the Pinpoint Test should show

| Test | If you see | Cause | Fix |
|---|---|---|---|
| **Push forward 24″** | X ≈ +24, Y ≈ 0 | Correct | Nothing to fix. |
| | X ≈ **−24** | X pod mounted backwards | X direction → `REVERSED` |
| | X ≈ 0, but **Y changes** | X and Y pods swapped | Swap the two pod cables at the Pinpoint |
| | X ≈ 0 and Y ≈ 0 | X pod unplugged or not touching the floor | Check the cable and mount. Status may say `FAULT_X_POD_NOT_DETECTED`. |
| | X ≈ 15 or ≈ 38 | Wrong pod type | Switch between `goBILDA_4_BAR_POD` and `goBILDA_SWINGARM_POD` |
| **Push left 24″** | Y ≈ +24, X ≈ 0 | Correct | Nothing to fix. |
| | Y ≈ **−24** | Y pod mounted backwards | Y direction → `REVERSED` |
| | Y ≈ 0, but **X changes** | X and Y pods swapped | Swap the two pod cables at the Pinpoint |
| | Y ≈ 0 and X ≈ 0 | Y pod unplugged or not touching the floor | Check the cable and mount. Status may say `FAULT_Y_POD_NOT_DETECTED`. |
| **Turn left ¼ turn** | Heading ≈ +90 | Correct | Nothing to fix. |
| | Heading ≈ **−90** | Pinpoint mounted upside down | Remount it sticker side up |
| | Heading barely changes | Pinpoint not reporting | Check the status line, the I2C cable and the configuration |
| **Spin in place 3×** | X and Y stay within 1–2″ | Correct | Nothing to fix. |
| | X or Y wanders several inches | A pod offset has the wrong sign or value | Remeasure. If it's still off, flip the sign of one offset at a time and spin again. |

The StarterBot can't be pushed sideways easily, so check its Y pod by sliding the robot left by hand. Its `drive()` doesn't use Y, but `sideSlip` telemetry does.

## Robot keeps going and never stops

| What you see | Most likely cause | How to confirm | Fix |
|---|---|---|---|
| Strafes and never stops | Y pod mounted backwards | Pinpoint Test: push left 24″. Y reads about −24. | Y direction → `REVERSED`, in all three files |
| Drives forward and never stops | X pod mounted backwards | Pinpoint Test: push forward 24″. X reads about −24. | X direction → `REVERSED`, in all three files |
| Drives the wrong way (backward on `drive(24, ...)`), then keeps going | Drive motors reversed | Pinpoint Test passes, but the Motor Direction Test shows wheels spinning backward | Flip those motors' directions, in all four files |
| Turns and never stops, or spins past the target | Pinpoint upside down, or motor directions wrong | Pinpoint Test: turn left. Heading goes *down* = Pinpoint. Heading goes up = motors. | Remount sticker side up, or fix motor directions |
| Keeps strafing, but drifts forward or spins instead of sliding | Front and back motors swapped on one side, or mecanum rollers wrong | Motor Direction Test: the wrong wheel spins for a button. From above, rollers should form an X. | Swap the motor cables or config ports, or swap the wheels |
| Keeps going with readings stuck near 0 | Pod unplugged, off the floor, or X and Y swapped | Pinpoint Test: the reading for that direction doesn't change | Fix the cable, mount or ports |
| Passes the Pinpoint Test, still runs away in autonomous | A setting was fixed in one file but not the others | Compare `setEncoderDirections` and motor directions across the files | Make them match everywhere |

## Robot moves, but wrong

| What you see | Most likely cause | Fix |
|---|---|---|
| Robot jumps to full speed when told to go slow | A drive motor's encoder cable is unplugged | Plug in all motor encoder cables |
| One wheel never turns | Motor in the wrong port, or configured as the wrong motor type | Check the cable and the configuration |
| Drives in a curve, or veers off the line | One motor reversed, or a loose wheel or set screw | Motor Direction Test, then check each wheel's hub |
| `turnTo(0, ...)` doesn't face where the robot started | No `resetHeading()` after `waitForStart()`, or the robot was bumped during INIT | Call `resetHeading()` right after start. Don't touch the robot during INIT. |
| Distances come out a few inches short or long every time | Wrong pod type, or a pod that slips or wobbles | Pinpoint Test: a 24″ push should read 24 ± 0.5 |
| Accurate at first, worse after every turn | Pod offsets wrong | Pinpoint Test spin check, then remeasure |
| Robot stops just short and hums, and the call never returns | Gain too low to push the last fraction of an inch | Raise that axis's `_GAIN` a little (0.03 → 0.035), or lower the hold time |
| Robot overshoots and rocks at the end | Gain or power too high | Lower that axis's `_GAIN` (0.03 → 0.025) or the power |
| Turns end a degree or two off | Turn gain a little low, or a low battery | Raise `YAW_GAIN` (0.018 → 0.022), and charge the battery |

## Install, configuration and INIT

| What you see | Most likely cause | Fix |
|---|---|---|
| Build fails | Another file has an error, or two files define the same class | Read the first error and fix or delete the file it names. Don't keep Phil's original `SimplifiedOdometryRobot.java` and SPOT's on the same Control Hub. |
| `cannot find symbol` on a `GoBildaPinpointDriver` method | SDK older than 10.3, or an old goBILDA driver file in `teamcode` | Update the SDK, and delete any `GoBildaPinpointDriver.java` from `teamcode` |
| Two Pinpoint entries in the configuration's I2C device list | Old goBILDA driver files left in OnBot Java | Delete them, rebuild, and pick **goBILDA Pinpoint Odometry Computer** |
| `Unable to find a hardware device with the name ...` | A configuration name doesn't match, or the wrong configuration is active | Mecanum: `drive_leftfront_motor`, `drive_rightfront_motor`, `drive_leftback_motor`, `drive_rightback_motor`, `odom_pinpoint_sensor`. StarterBot: `left_drive`, `right_drive`, `odo`. |
| Heading creeps while the robot sits still | The robot moved during INIT while the gyro calibrated | Stop, set the robot down still, and INIT again |

## Pinpoint status codes

`PinpointTest` shows the status on the Driver Station. The Pinpoint's light shows the same thing.

| Status | Light | Meaning | Fix |
|---|---|---|---|
| `READY` | Green | Working normally | Nothing to fix. |
| `NOT_READY` | Red | Still starting up after power-on | Wait a second. If it stays red, check the reset button isn't stuck down. |
| `CALIBRATING` | Red | Calibrating the gyro (about 0.25 s) | Keep the robot still. Normal during INIT. |
| `FAULT_NO_PODS_DETECTED` | Purple | Neither pod is connected | Check both pod cables |
| `FAULT_X_POD_NOT_DETECTED` | Blue | X pod not connected | Check the X pod cable. Swap the X and Y cables to tell a bad pod from a bad cable. |
| `FAULT_Y_POD_NOT_DETECTED` | Orange | Y pod not connected | Check the Y pod cable, same swap test |

Status meanings and light colors are from goBILDA's [Pinpoint user guide](https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf). If the light stays purple, blue or orange no matter which pods are plugged in, contact goBILDA support.
