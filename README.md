# Arduino Woz Main Controller
Dedicated firmware to manage the motor drivers and the IMU.

## Request Packet Format

| Start Flag | Command | Payload Size |
|------------|---------|--------------|
| 1 byte     | 1 byte  | x bytes      |

## Response Packet Format

| Start Flag 1 | Start Flag 2 | Command | Response Data Length | Response |
|--------------|--------------|---------|----------------------|----------|
| `0xA3`       | `0x3A`       | 1 byte  | 1 byte               | x bytes  |

## Requests Overview

| Request   | Value  | Payload                                                             |
|-----------|--------|---------------------------------------------------------------------|
| FORWARD   | `0x10` | speed (1 byte), distance (1 byte) [optional]                        |
| REVERSE   | `0x11` | speed (1 byte), distance (1 byte) [optional]                        |
| ROTATE    | `0x12` | speed (1 byte), angle (1 byte), direction (1 bit)                   |
| TURN      | `0x13` | speed (1 byte), angle (1 byte), radius (1 byte), direction (1 bit) |
| DRIVE     | `0x14` | speed left (1 byte), speed right (1 byte)                           |
| STOP      | `0x15` | hard stop (1 bit)                                                   |
| RESET_IMU | `0x20` | N/A                                                                 |

## Motors Forward Request
Request: `0xA3` `0x10` `0x[speed]` `0x[distance]`

Moves the robot forward in a straight line.

## Motors Reverse Request
Request: `0xA3` `0x11` `0x[speed]` `0x[distance]`

Moves the robot backward in a straight line.

## Motors Rotate Request
Request: `0xA3` `0x12` `0x[speed]` `0x[angle]` `0x[0, 0, 0, 0, 0, 0, 0, direction]`

Rotates the robot to a given angle in a given direction (1 is left, 0 is right).

## Motors Turn Request
Request: `0xA3` `0x13` `0x[speed]` `0x[angle]` `0x[radius]` `0x[0, 0, 0, 0, 0, 0, 0, direction]`

Turns the robot while driving with a given radius in a given direction (1 is left, 0 is right).

## Motors Drive Request
Request: `0xA3` `0x14` `0x[speed left]` `0x[speed right]`

This freestyle mode let's you set the. left and right speed separately. Mostly used for line following.

## Motors Stop Request
Request: `0xA3` `0x15` `0x[0, 0, 0, 0, 0, 0, 0, hard]`

Stops both motors. When the hard bit is set to 1 it wil stop the motors immediately.

## Reset IMU Request
Request: `0xA3 0x20`

Resets the IMU.

### Ready Response
**Response:** `0xA3` `0x3A` `0xFF` `0x01` `0x[imu detection flag]`

This response will be sent when the main controller is ready to be controlled. The imu detection flag describes whether the IMU was found.

### Odometry Response
**Response:** `0xA3` `0x3A` `0x30` `0x04` `0x[num ticks left]` `0x[num ticks right]` `0x[heading 15:8]` `0x[heading 7:0]`

This response will be sent 50 times per second. The heading will need to be divided by 100 to get the correct angle up to two decimals.
