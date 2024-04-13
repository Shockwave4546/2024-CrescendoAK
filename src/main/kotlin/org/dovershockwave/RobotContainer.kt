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

package org.dovershockwave

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.dovershockwave.subsystem.swerve.SwerveConstants
import org.dovershockwave.subsystem.swerve.SwerveSubsystem
import org.dovershockwave.subsystem.swerve.gyro.GyroIONavX
import org.dovershockwave.subsystem.swerve.gyro.GyroIOSim
import org.dovershockwave.subsystem.swerve.module.ModuleIOSim
import org.dovershockwave.subsystem.swerve.module.ModuleIOSpark

object RobotContainer {
  val swerve: SwerveSubsystem
  val driverController = CommandXboxController(GlobalConstants.DRIVER_CONTROLLER_PORT)
  val operatorController = CommandXboxController(GlobalConstants.OPERATOR_CONTROLLER_PORT)

  init {
    when (GlobalConstants.robotType) {
      RobotType.REAL -> {
        swerve = SwerveSubsystem(
          ModuleIOSpark(SwerveConstants.FRONT_LEFT_DRIVING_CAN_ID, SwerveConstants.FRONT_LEFT_TURNING_CAN_ID, SwerveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET),
          ModuleIOSpark(SwerveConstants.FRONT_RIGHT_DRIVING_CAN_ID, SwerveConstants.FRONT_RIGHT_TURNING_CAN_ID, SwerveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET),
          ModuleIOSpark(SwerveConstants.BACK_LEFT_DRIVING_CAN_ID, SwerveConstants.BACK_LEFT_TURNING_CAN_ID, SwerveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET),
          ModuleIOSpark(SwerveConstants.BACK_RIGHT_DRIVING_CAN_ID, SwerveConstants.BACK_RIGHT_TURNING_CAN_ID, SwerveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET),
          GyroIONavX()
        )
      }

      RobotType.SIM -> {
        swerve = SwerveSubsystem(
          ModuleIOSim(SwerveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET),
          ModuleIOSim(SwerveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET),
          ModuleIOSim(SwerveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET),
          ModuleIOSim(SwerveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET),
          GyroIOSim() // TODO: This is fake for now.
        )
      }
    }

    configureBindings()

    DriverStation.silenceJoystickConnectionWarning(true)
  }

  private fun configureBindings() {

  }
}