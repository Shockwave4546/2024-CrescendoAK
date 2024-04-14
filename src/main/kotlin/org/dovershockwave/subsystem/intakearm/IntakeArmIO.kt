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

package org.dovershockwave.subsystem.intakearm

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IntakeArmIO {
  class IntakeArmIOInputs : LoggableInputs {
    var angle = 0.0
    var appliedVolts = 0.0
    var current = 0.0
    var temp = 0.0

    override fun toLog(table: LogTable) {
      table.put("Angle", angle)
      table.put("Applied Volts", appliedVolts)
      table.put("Current", current)
      table.put("Temp", temp)
    }

    override fun fromLog(table: LogTable) {
      angle = table.get("Angle", angle)
      appliedVolts = table.get("Applied Volts", appliedVolts)
      current = table.get("Current", current)
      temp = table.get("Temp", temp)
    }
  }

  fun updateInputs(inputs: IntakeArmIOInputs)

  fun setAngleSetpoint(angle: Double)
}