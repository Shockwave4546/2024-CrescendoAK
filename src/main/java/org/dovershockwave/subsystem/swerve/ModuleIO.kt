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

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.littletonrobotics.junction.AutoLog

interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    var drivePosition: Double = 0.0 // m
    var driveVelocity: Double = 0.0 // m/s
    var driveAppliedVolts: Double = 0.0
    var driveCurrent: Double = 0.0
    var driveTemp: Double = 0.0 // Celsius

    var rotPosition: Rotation2d = Rotation2d()
    var rotVelocity: Double = 0.0 // rad/s
    var rotAppliedVolts: Double = 0.0
    var rotCurrent: Double = 0.0
    var rotTemp: Double = 0.0 // Celsius
  }

  fun updateInputs(inputs: ModuleIOInputs)

  fun getState(): SwerveModuleState

  fun getPosition(): SwerveModulePosition

  fun setDesiredState(desiredState: SwerveModuleState)

  fun resetDriveEncoder()
}