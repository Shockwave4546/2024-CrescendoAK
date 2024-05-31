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

package org.dovershockwave.auto

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import org.dovershockwave.Tab
import org.dovershockwave.subsystem.intake.IntakeSubsystem
import org.dovershockwave.subsystem.intakearm.ArmState
import org.dovershockwave.subsystem.intakearm.IntakeArmSubsystem
import org.dovershockwave.subsystem.shooter.ShooterState
import org.dovershockwave.subsystem.shooter.ShooterSubsystem
import org.dovershockwave.subsystem.shooterwrist.ShooterWristSubsystem
import org.dovershockwave.subsystem.shooterwrist.WristState
import org.dovershockwave.subsystem.swerve.SwerveConstants
import org.dovershockwave.subsystem.swerve.SwerveSubsystem
import org.dovershockwave.subsystem.vision.VisionSubsystem
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

class AutoManager(swerve: SwerveSubsystem, shooter: ShooterSubsystem, wrist: ShooterWristSubsystem, arm: IntakeArmSubsystem, intake: IntakeSubsystem, vision: VisionSubsystem) {
  private val chooser: LoggedDashboardChooser<Command>

  init {
    AutoBuilder.configureHolonomic(
      vision::getPose2d,
      vision::resetPose,
      swerve::getRelativeChassisSpeed,
      swerve::driveAutonomous,
      HolonomicPathFollowerConfig(
        PIDConstants(AutoConstants.DRIVING_GAINS.p, AutoConstants.DRIVING_GAINS.i, AutoConstants.DRIVING_GAINS.d),
        PIDConstants(AutoConstants.TURNING_GAINS.p, AutoConstants.TURNING_GAINS.i, AutoConstants.TURNING_GAINS.d),
        SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
        SwerveConstants.WHEEL_BASE / 2.0,
        ReplanningConfig()
      ),
      this::shouldFlipPath,
      swerve
    )


    // Note: Named commands must be registered before the creation of any PathPlanner Autos or Paths.
    NamedCommands.registerCommand("RampClose", InstantCommand({ shooter.setDesiredState(ShooterState.SUBWOOFER) }, shooter))
    NamedCommands.registerCommand("IntakeNote", AutoIntakeCommand(arm, intake))
    NamedCommands.registerCommand("ShootClose", AutoShootCloseCommand(intake, shooter, arm, wrist))
    NamedCommands.registerCommand("StopShooter", InstantCommand({ shooter.setDesiredState(ShooterState.STOPPED) }, shooter))
    NamedCommands.registerCommand("IntakeHome", InstantCommand({ arm.setDesiredState(ArmState.HOME) }, arm))
    NamedCommands.registerCommand("ShootInterpolated", InstantCommand({ shooter.setDesiredState(ShooterState.INTERPOLATED) }, shooter))
    NamedCommands.registerCommand("WristHome", InstantCommand({ wrist.setDesiredState(WristState.HOME) }, wrist))
    NamedCommands.registerCommand("AimAndShoot", AutoAimAndShootCommand(intake, shooter, arm, wrist, swerve, vision))
    NamedCommands.registerCommand("WaitForIntake", WaitUntilCommand { arm.atDesiredState() })
    NamedCommands.registerCommand("DropIntake", InstantCommand({ arm.setDesiredState(ArmState.FLOOR) }, arm))

    this.chooser = LoggedDashboardChooser("Autonomous", AutoBuilder.buildAutoChooser())
    Tab.MATCH.add("Autonomous", chooser.sendableChooser).withSize(3, 3)
  }

  /**
   * Red = flip, since PathPlanner uses blue as the default wall.
   *
   * @return whether the autonomous path should be flipped dependent on the alliance color.
   */
  private fun shouldFlipPath() = DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get() == DriverStation.Alliance.Red

  /**
   * Schedules the selected autonomous mode.
   */
  fun scheduleRoutine() = chooser.get().schedule()
}