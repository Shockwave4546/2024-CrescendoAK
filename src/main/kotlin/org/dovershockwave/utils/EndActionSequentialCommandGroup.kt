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

package org.dovershockwave.utils

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

open class EndActionSequentialCommandGroup(private val endAction: Command) : Command() {
  private val commands: MutableList<Command> = ArrayList()
  private var currentCommandIndex = -1
  private var runWhenDisabled = true
  private var interruptBehavior = InterruptionBehavior.kCancelIncoming

  fun addCommands(vararg commands: Command) {
    check(currentCommandIndex == -1) { "Commands cannot be added to a composition while it's running" }

    CommandScheduler.getInstance().registerComposedCommands(*commands)

    for (command in commands) {
      this.commands.add(command)
      m_requirements.addAll(command.requirements)
      runWhenDisabled = runWhenDisabled and command.runsWhenDisabled()
      if (command.interruptionBehavior == InterruptionBehavior.kCancelSelf) {
        interruptBehavior = InterruptionBehavior.kCancelSelf
      }
    }
  }


  override fun initialize() {
    currentCommandIndex = 0

    if (commands.isNotEmpty()) {
      commands[0].initialize()
    }
  }

  override fun execute() {
    if (commands.isEmpty()) {
      return
    }

    val currentCommand = commands[currentCommandIndex]

    currentCommand.execute()
    if (currentCommand.isFinished) {
      currentCommand.end(false)
      currentCommandIndex++
      if (currentCommandIndex < commands.size) {
        commands[currentCommandIndex].initialize()
      }
    }
  }

  override fun end(interrupted: Boolean) {
    if (interrupted && commands.isNotEmpty() && currentCommandIndex > -1 && currentCommandIndex < commands.size) {
      commands[currentCommandIndex].end(true)
    }

    endAction.schedule()
    currentCommandIndex = -1
  }

  override fun isFinished(): Boolean {
    return currentCommandIndex == commands.size
  }

  override fun runsWhenDisabled(): Boolean {
    return runWhenDisabled
  }

  override fun getInterruptionBehavior(): InterruptionBehavior {
    return interruptBehavior
  }
}