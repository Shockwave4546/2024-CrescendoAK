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

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IntakeIO {
  class IntakeIOInputs : LoggableInputs {
    var hasNote = false
    var dutyCycle = 0.0
    var appliedVolts = 0.0
    var current = 0.0
    var temp = 0.0

    override fun toLog(table: LogTable) {
      table.put("1.HasNote", hasNote)
      table.put("2.DutyCycle", dutyCycle)
      table.put("3.AppliedVolts", appliedVolts)
      table.put("4.Current", current)
      table.put("5.Temp", temp)
    }

    override fun fromLog(table: LogTable) {
      hasNote = table.get("1.HasNote", hasNote)
      dutyCycle = table.get("2.DutyCycle", dutyCycle)
      appliedVolts = table.get("3.AppliedVolts", appliedVolts)
      current = table.get("4.Current", current)
      temp = table.get("5.Temp", temp)
    }
  }

  fun updateInputs(inputs: IntakeIOInputs)

  fun setDutyCycle(dutyCycle: Double)

  fun hasNote(): Boolean
}