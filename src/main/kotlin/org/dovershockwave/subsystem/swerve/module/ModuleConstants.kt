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

package org.dovershockwave.subsystem.swerve.module

import org.dovershockwave.utils.PIDGains
import org.dovershockwave.utils.PositionConversionFactor

class ModuleConstants {
  companion object {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    const val DRIVING_MOTOR_PINION_TEETH = 13

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    const val TURNING_ENCODER_INVERTED = true
    const val DRIVE_DIRECTION_INVERTED = true

    // Calculations required for driving motor conversion factors and feed forward
    const val DRIVING_MOTOR_FREE_SPEED_RPS = org.dovershockwave.MotorConstants.NEO_FREE_SPEED_RPM / 60
    const val WHEEL_DIAMETER_METERS = 0.071
    const val WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    const val DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15)
    const val DRIVE_WHEEL_FREE_SPEED_RPS = ((DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION)

    val DRIVING_ENCODER_POSITION_FACTOR = PositionConversionFactor((WHEEL_DIAMETER_METERS * Math.PI) / DRIVING_MOTOR_REDUCTION) // Meters
    const val DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI) / DRIVING_MOTOR_REDUCTION) / 60.0 // Meters per second

    val TURNING_ENCODER_POSITION_FACTOR = PositionConversionFactor(PositionConversionFactor.ConversionType.RADIANS) // Radians
    const val TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0 // Radians per second

    const val TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0.0 // Radians
    const val TURNING_ENCODER_POSITION_PID_MAX_INPUT = 2 * Math.PI // Radians

    const val VOLTAGE_COMPENSATION = 12.0

    val DRIVING_GAINS = PIDGains(0.15, 0.0, 0.02, 1 / DRIVE_WHEEL_FREE_SPEED_RPS)
    const val DRIVING_MIN_OUTPUT = -1.0
    const val DRIVING_MAX_OUTPUT = 1.0

    val TURNING_GAINS = PIDGains(0.5)
    const val TURNING_MIN_OUTPUT = -1.0
    const val TURNING_MAX_OUTPUT = 1.0
  }
}