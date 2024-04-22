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
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.dovershockwave.auto.AutoManager
import org.dovershockwave.commands.*
import org.dovershockwave.subsystem.intake.IntakeConstants
import org.dovershockwave.subsystem.intake.IntakeIOSim
import org.dovershockwave.subsystem.intake.IntakeIOSpark
import org.dovershockwave.subsystem.intake.IntakeSubsystem
import org.dovershockwave.subsystem.intake.commands.FeedShooterCommand
import org.dovershockwave.subsystem.intakearm.*
import org.dovershockwave.subsystem.led.LEDSubsystem
import org.dovershockwave.subsystem.pose.PoseEstimatorIOReal
import org.dovershockwave.subsystem.pose.PoseEstimatorSubsystem
import org.dovershockwave.subsystem.pose.commands.ResetFieldCentricDriveCommand
import org.dovershockwave.subsystem.shooter.ShooterConstants
import org.dovershockwave.subsystem.shooter.ShooterIOSim
import org.dovershockwave.subsystem.shooter.ShooterIOSpark
import org.dovershockwave.subsystem.shooter.ShooterSubsystem
import org.dovershockwave.subsystem.shooterwrist.*
import org.dovershockwave.subsystem.swerve.SwerveConstants
import org.dovershockwave.subsystem.swerve.SwerveSubsystem
import org.dovershockwave.subsystem.swerve.commands.SetMaxSpeedCommand
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
  val poseEstimator: PoseEstimatorSubsystem? // TODO: 4/14/2024  
  val led: LEDSubsystem? // TODO: 4/13/2024 idk 
  val driverController = CommandXboxController(GlobalConstants.DRIVER_CONTROLLER_PORT)
  val operatorController = CommandXboxController(GlobalConstants.OPERATOR_CONTROLLER_PORT)
  val autoManager: AutoManager?

  init {
    when (GlobalConstants.ROBOT_TYPE) {
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
        poseEstimator = PoseEstimatorSubsystem(PoseEstimatorIOReal(), swerve)
        led = LEDSubsystem()
        autoManager = AutoManager(swerve, shooter, wrist, arm, intake, poseEstimator)
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
        poseEstimator = null
        led = null
        autoManager = null
      }
    }

    configureBindings()

    DriverStation.silenceJoystickConnectionWarning(true)
  }

  fun isRedAlliance() = DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get() == DriverStation.Alliance.Red

  private fun configureBindings() {
    driverController.b().onTrue(ResetFieldCentricDriveCommand(swerve, poseEstimator!!)) // FIXME:  
    driverController.x().onTrue(InstantCommand({ swerve.toggleX() }, swerve))
    driverController.leftBumper().whileTrue(SetMaxSpeedCommand(swerve, 0.2, 0.2))
    driverController.rightBumper().whileTrue(SetMaxSpeedCommand(swerve, 0.4, 0.4))

    operatorController.povUp().onTrue(InstantCommand({ arm.setDesiredState(ArmState.HOME) }, arm))
    operatorController.povDown().onTrue(InstantCommand({ arm.setDesiredState(ArmState.FLOOR) }, arm))
    operatorController.povRight().onTrue(FeedShooterCommand(intake).withTimeout(0.25))

    operatorController.leftBumper().onTrue(ResetRobotStateCommand(shooter, intake, arm, wrist))

    operatorController.rightTrigger().onTrue(InstantCommand({ wrist.setDesiredState(WristState.SPIT) }, wrist))
    operatorController.rightBumper().onTrue(FullSpitCommand(intake, shooter, arm, wrist))
    operatorController.a().toggleOnTrue(FullShootCloseCommand(intake, shooter, arm, wrist))
    operatorController.b().toggleOnTrue(FullSpitCommand(intake, shooter, arm, wrist))
    operatorController.x().toggleOnTrue(FullShootAmpCommand(intake, shooter, arm, wrist))
    operatorController.y().toggleOnTrue(FullIntakeCommand(arm, intake))
  }
}