package org.dovershockwave.commands

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.dovershockwave.subsystem.intake.IntakeSubsystem
import org.dovershockwave.subsystem.intake.commands.IntakeNoteCommand
import org.dovershockwave.subsystem.intakearm.ArmState
import org.dovershockwave.subsystem.intakearm.IntakeArmSubsystem

class FullIntakeCommand(arm: IntakeArmSubsystem, intake: IntakeSubsystem) : SequentialCommandGroup() {
  init {
    addCommands(
      InstantCommand({ arm.setDesiredState(ArmState.FLOOR) }, arm),
      IntakeNoteCommand(intake).until(intake::hasNote),
      InstantCommand({ arm.setDesiredState(ArmState.HOME) }, arm)
    )

    addRequirements(intake, arm)
  }
}