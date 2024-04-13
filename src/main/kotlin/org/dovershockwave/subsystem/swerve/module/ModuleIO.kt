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

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ModuleIO {
  class ModuleIOInputs : LoggableInputs {
    var drivePosition = 0.0 // m
    var driveVelocity = 0.0 // m/s
    var driveAppliedVolts = 0.0
    var driveCurrent = 0.0
    var driveTemp = 0.0 // Celsius

    var rotPosition = Rotation2d()
    var rotVelocity = 0.0 // rad/s
    var rotAppliedVolts = 0.0
    var rotCurrent = 0.0
    var rotTemp = 0.0 // Celsius

    override fun toLog(table: LogTable) {
      table.put("Drive Position", drivePosition)
      table.put("Drive Velocity", driveVelocity)
      table.put("Drive Applied Volts", driveAppliedVolts)
      table.put("Drive Current", driveCurrent)
      table.put("Drive Temp", driveTemp)

      table.put("Rot Position", rotPosition.radians)
      table.put("Rot Velocity", rotVelocity)
      table.put("Rot Applied Volts", rotAppliedVolts)
      table.put("Rot Current", rotCurrent)
      table.put("Rot Temp", rotTemp)
    }

    override fun fromLog(table: LogTable) {
      drivePosition = table.get("Drive Position", drivePosition)
      driveVelocity = table.get("Drive Velocity", driveVelocity)
      driveAppliedVolts = table.get("Drive Applied Volts", driveAppliedVolts)
      driveCurrent = table.get("Drive Current", driveCurrent)
      driveTemp = table.get("Drive Temp", driveTemp)

      rotPosition = Rotation2d(table.get("Rot Position", rotPosition.radians))
      rotVelocity = table.get("Rot Velocity", rotVelocity)
      rotAppliedVolts = table.get("Rot Applied Volts", rotAppliedVolts)
      rotCurrent = table.get("Rot Current", rotCurrent)
      rotTemp = table.get("Rot Temp", rotTemp)
    }
  }

  fun updateInputs(inputs: ModuleIOInputs)

  fun getState(): SwerveModuleState

  fun getPosition(): SwerveModulePosition

  fun setDesiredState(desiredState: SwerveModuleState)

  fun resetDriveEncoder()
}