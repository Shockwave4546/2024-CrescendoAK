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

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import org.dovershockwave.subsystem.intake.IntakeSubsystem
import org.dovershockwave.subsystem.intake.commands.FeedShooterCommand
import org.dovershockwave.subsystem.intakearm.ArmState
import org.dovershockwave.subsystem.intakearm.IntakeArmSubsystem
import org.dovershockwave.subsystem.shooter.ShooterSubsystem
import org.dovershockwave.subsystem.shooterwrist.ShooterWristSubsystem
import org.dovershockwave.subsystem.shooterwrist.WristState

class AutoShootCloseCommand(intake: IntakeSubsystem, shooter: ShooterSubsystem, arm: IntakeArmSubsystem, wrist: ShooterWristSubsystem) : SequentialCommandGroup() {
  init {
    addCommands(
      InstantCommand({ wrist.setDesiredState(WristState.SUBWOOFER) }, wrist),
      InstantCommand({ arm.setDesiredState(ArmState.HOME) }, arm),
      WaitUntilCommand(shooter::atDesiredState),
      WaitUntilCommand(arm::atDesiredState),
      FeedShooterCommand(intake).withTimeout(0.25)
    )

    addRequirements(intake, arm, wrist)
  }
}