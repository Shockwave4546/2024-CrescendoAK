package org.dovershockwave.commands

import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import org.dovershockwave.Tab
import org.dovershockwave.shuffleboard.ShuffleboardBoolean
import org.dovershockwave.subsystem.intake.IntakeSubsystem
import org.dovershockwave.subsystem.intakearm.IntakeArmSubsystem
import org.dovershockwave.subsystem.shooter.ShooterSubsystem
import org.dovershockwave.subsystem.shooterwrist.ShooterWristSubsystem
import org.dovershockwave.subsystem.swerve.SwerveSubsystem
import org.dovershockwave.subsystem.vision.VisionSubsystem
import org.dovershockwave.utils.EndActionSequentialCommandGroup

class AimAndShootCommand(intake: IntakeSubsystem, shooter: ShooterSubsystem, arm: IntakeArmSubsystem, wrist: ShooterWristSubsystem, swerve: SwerveSubsystem, vision: VisionSubsystem) : EndActionSequentialCommandGroup(ResetRobotStateCommand(shooter, intake, arm, wrist)){
  init {
    addCommands(
      ParallelCommandGroup(
        ConditionalCommand(TrackSpeakerCommand(swerve, vision), InstantCommand()) { AUTO_AIM_ON_SHOOT.get() },
        FullShootInterpolatedCommand(intake, shooter, arm, wrist)
      )
    )

    addRequirements(shooter, intake, arm, wrist, swerve, vision)
  }

  companion object {
    val AUTO_AIM_ON_SHOOT = ShuffleboardBoolean(Tab.MATCH, "Auto Aim on Shoot", false).withSize(3, 3)
  }
}