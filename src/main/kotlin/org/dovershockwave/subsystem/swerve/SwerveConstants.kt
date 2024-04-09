/*
 * Copyright (c) 2023 FRC Team 4546
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.dovershockwave.subsystem.swerve

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import org.dovershockwave.subsystem.swerve.module.ModulePosition

class SwerveConstants {
  companion object {
    const val INVERT_DRIVING_DIRECTION = false

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    const val MAX_SPEED_METERS_PER_SECOND = 4.65
    const val MAX_ANGULAR_SPEED = 2 * Math.PI // radians per second

    // Distance between centers of right and left wheels on robot
    const val TRACK_WIDTH = 0.545 // m

    // Distance between front and back wheels on robot
    const val WHEEL_BASE = 0.545 // m

    val MODULE_TRANSLATIONS: Map<ModulePosition, Translation2d> = mapOf(
      ModulePosition.FRONT_LEFT to Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      ModulePosition.FRONT_RIGHT to Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      ModulePosition.BACK_LEFT to Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
      ModulePosition.BACK_RIGHT to Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    )

    val DRIVE_KINEMATICS: SwerveDriveKinematics = SwerveDriveKinematics(
      MODULE_TRANSLATIONS[ModulePosition.FRONT_LEFT],
      MODULE_TRANSLATIONS[ModulePosition.FRONT_RIGHT],
      MODULE_TRANSLATIONS[ModulePosition.BACK_LEFT],
      MODULE_TRANSLATIONS[ModulePosition.BACK_RIGHT]
    )

    // Angular offsets of the modules relative to the chassis in radians
    const val FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI
    const val FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2
    const val BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2
    const val BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = 0.0

    // Driving Motor Prefix = 1x
    const val FRONT_LEFT_DRIVING_CAN_ID = 10
    const val FRONT_RIGHT_DRIVING_CAN_ID = 11
    const val BACK_LEFT_DRIVING_CAN_ID = 13
    const val BACK_RIGHT_DRIVING_CAN_ID = 14

    // Turning Motor Prefix = 2x
    const val FRONT_LEFT_TURNING_CAN_ID = 20
    const val FRONT_RIGHT_TURNING_CAN_ID = 21
    const val BACK_LEFT_TURNING_CAN_ID = 23
    const val BACK_RIGHT_TURNING_CAN_ID = 24

    const val GYRO_REVERSED = true

    const val DEFAULT_DRIVE_SPEED_MULTIPLIER = 0.85
    const val DEFAULT_ROT_SPEED_MULTIPLIER = 0.85
  }
}