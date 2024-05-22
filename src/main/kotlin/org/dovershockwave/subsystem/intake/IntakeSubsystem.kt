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

package org.dovershockwave.subsystem.intake

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.dovershockwave.shuffleboard.ShuffleboardBoolean
import org.dovershockwave.subsystem.intake.commands.IdleIntakeCommand
import org.littletonrobotics.junction.Logger

class IntakeSubsystem(private val intake: IntakeIO) : SubsystemBase() {
  private val inputs = IntakeIO.IntakeIOInputs()
  private var desiredState = IntakeState.IDLE
  private val runIdle = ShuffleboardBoolean(Shuffleboard.getTab("Intake"), "Idle Speed", true).withSize(3, 3)

  init {
    defaultCommand = IdleIntakeCommand(this)
  }

  override fun periodic() {
    intake.updateInputs(inputs)

    val key = "Intake"
    Logger.processInputs(key, inputs)
    Logger.recordOutput("$key/DesiredState/Name", desiredState.name)
    Logger.recordOutput("$key/DesiredState/Duty Cycle", desiredState.dutyCycle)
    intake.setDutyCycle(desiredState.dutyCycle)
  }

  fun setDesiredState(desiredState: IntakeState) {
    this.desiredState = desiredState
  }

  fun isIdle() = runIdle.get()

  fun hasNote() = intake.hasNote()
}