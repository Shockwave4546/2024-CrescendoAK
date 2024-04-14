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
import org.dovershockwave.subsystem.intake.*
import org.dovershockwave.subsystem.intakearm.IntakeArmConstants
import org.dovershockwave.subsystem.intakearm.IntakeArmIOSim
import org.dovershockwave.subsystem.intakearm.IntakeArmIOSpark
import org.dovershockwave.subsystem.intakearm.IntakeArmSubsystem
import org.dovershockwave.subsystem.led.LEDSubsystem
import org.dovershockwave.subsystem.shooter.ShooterConstants
import org.dovershockwave.subsystem.shooter.ShooterIOSim
import org.dovershockwave.subsystem.shooter.ShooterIOSpark
import org.dovershockwave.subsystem.shooter.ShooterSubsystem
import org.dovershockwave.subsystem.shooterwrist.ShooterWristIOSim
import org.dovershockwave.subsystem.shooterwrist.ShooterWristIOSpark
import org.dovershockwave.subsystem.shooterwrist.ShooterWristSubsystem
import org.dovershockwave.subsystem.shooterwrist.WristConstants
import org.dovershockwave.subsystem.swerve.SwerveConstants
import org.dovershockwave.subsystem.swerve.SwerveSubsystem
import org.dovershockwave.subsystem.swerve.gyro.GyroIONavX
import org.dovershockwave.subsystem.swerve.gyro.GyroIOSim
import org.dovershockwave.subsystem.swerve.module.ModuleIOSim
import org.dovershockwave.subsystem.swerve.module.ModuleIOSpark

object RobotContainer {
  val swerve: SwerveSubsystem
  val wrist: ShooterWristSubsystem
  val arm: IntakeArmSubsystem
  val shooter: ShooterSubsystem
  val intake: IntakeSubsystem
  val led: LEDSubsystem? // TODO: 4/13/2024 idk 
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

        wrist = ShooterWristSubsystem(ShooterWristIOSpark(WristConstants.MOTOR_CAN_ID))
        arm = IntakeArmSubsystem(IntakeArmIOSpark(IntakeArmConstants.MOTOR_CAN_ID))
        shooter = ShooterSubsystem(ShooterIOSpark(ShooterConstants.BOTTOM_CAN_ID, ShooterConstants.TOP_CAN_ID))
        intake = IntakeSubsystem(IntakeIOSpark(IntakeConstants.MOTOR_CAN_ID))
        led = LEDSubsystem()
      }

      RobotType.SIM -> {
        swerve = SwerveSubsystem(
          ModuleIOSim(SwerveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET),
          ModuleIOSim(SwerveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET),
          ModuleIOSim(SwerveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET),
          ModuleIOSim(SwerveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET),
          GyroIOSim() // TODO: This is fake for now.
        )

        wrist = ShooterWristSubsystem(ShooterWristIOSim())
        arm = IntakeArmSubsystem(IntakeArmIOSim())
        shooter = ShooterSubsystem(ShooterIOSim())
        intake = IntakeSubsystem(IntakeIOSim())
        led = null
      }
    }

    configureBindings()

    DriverStation.silenceJoystickConnectionWarning(true)
  }

  private fun configureBindings() {

  }
}