package org.dovershockwave.auto

import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.dovershockwave.commands.AimAndShootCommand
import org.dovershockwave.commands.TrackSpeakerCommand
import org.dovershockwave.subsystem.intake.IntakeSubsystem
import org.dovershockwave.subsystem.intakearm.IntakeArmSubsystem
import org.dovershockwave.subsystem.shooter.ShooterSubsystem
import org.dovershockwave.subsystem.shooterwrist.ShooterWristSubsystem
import org.dovershockwave.subsystem.swerve.SwerveSubsystem
import org.dovershockwave.subsystem.vision.VisionSubsystem

class AutoAimAndShootCloseCommand(intake: IntakeSubsystem, shooter: ShooterSubsystem, arm: IntakeArmSubsystem, wrist: ShooterWristSubsystem, swerve: SwerveSubsystem, vision: VisionSubsystem) : SequentialCommandGroup() {
  init {
    addCommands(
      ParallelCommandGroup(
        ConditionalCommand(TrackSpeakerCommand(swerve, vision), InstantCommand()) { AimAndShootCommand.AUTO_AIM_ON_SHOOT.get() },
        AutoShootCloseCommand(intake, shooter, arm, wrist)
      )
    )

    addRequirements(shooter, intake, arm, wrist, swerve, vision)
  }
}