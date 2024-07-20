package org.dovershockwave.subsystem.intakearm.commands

import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import org.dovershockwave.subsystem.intakearm.ArmState
import org.dovershockwave.subsystem.intakearm.IntakeArmSubsystem

class SetIntakeStateCommand(arm: IntakeArmSubsystem, state: ArmState) : SequentialCommandGroup() {
  private val atDesiredState = arm.atDesiredState(state)

  init {
    addCommands(
      ConditionalCommand(InstantCommand(), InstantCommand({
        if (arm.atDesiredState(state)) return@InstantCommand
        val stateToSet = state.intermediateState ?: state
        arm.setDesiredState(stateToSet)
      })) { atDesiredState },

      ConditionalCommand(InstantCommand(), WaitUntilCommand { arm.atDesiredState() }.withTimeout(1.0)) { atDesiredState },

      InstantCommand({
        if (state.intermediateState == null) return@InstantCommand
        arm.setDesiredState(state)
      })
    )

    addRequirements(arm)
  }
}