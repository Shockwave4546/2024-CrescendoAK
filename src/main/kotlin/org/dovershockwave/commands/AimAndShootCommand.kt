package org.dovershockwave.commands

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
      TrackSpeakerCommand(swerve, vision),
      FullShootInterpolatedCommand(intake, shooter, arm, wrist, swerve, vision)
    )

    addRequirements(shooter, intake, arm, wrist, swerve, vision)
  }
}