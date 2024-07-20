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

package org.dovershockwave.subsystem.shooter

import com.revrobotics.REVLibError
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ShooterIO {
  class ShooterIOInputs : LoggableInputs {
    var bottomRPS = 0.0
    var bottomAppliedVolts = 0.0
    var bottomCurrent = 0.0
    var bottomTemp = 0.0

    var topRPS = 0.0
    var topAppliedVolts = 0.0
    var topCurrent = 0.0
    var topTemp = 0.0

    override fun toLog(table: LogTable) {
      table.put("Bottom/1.RPS", bottomRPS)
      table.put("Bottom/2.AppliedVolts", bottomAppliedVolts)
      table.put("Bottom/3.Current", bottomCurrent)
      table.put("Bottom/4.Temp", bottomTemp)

      table.put("Top/1.RPS", topRPS)
      table.put("Top/2.AppliedVolts", topAppliedVolts)
      table.put("Top/3.Current", topCurrent)
      table.put("Top/4.Temp", topTemp)
    }

    override fun fromLog(table: LogTable) {
      bottomRPS = table.get("Bottom/1.RPS", bottomRPS)
      bottomAppliedVolts = table.get("Bottom/2.AppliedVolts", bottomAppliedVolts)
      bottomCurrent = table.get("Bottom/3.Current", bottomCurrent)
      bottomTemp = table.get("Bottom/4.Temp", bottomTemp)

      topRPS = table.get("Top/1.RPS", topRPS)
      topAppliedVolts = table.get("Top/2.AppliedVolts", topAppliedVolts)
      topCurrent = table.get("Top/3.Current", topCurrent)
      topTemp = table.get("Top/4.Temp", topTemp)
    }
  }

  fun updateInputs(inputs: ShooterIOInputs)

  /**
   * Bottom motor
   */
  fun setBottomVelocitySetpoint(rps: Double): REVLibError

  fun setBotP(p: Double): REVLibError

  fun setBotI(i: Double): REVLibError

  fun setBotD(d: Double): REVLibError

  fun setBotFF(ff: Double): REVLibError

  fun stopBot()

  /**
   * Top motor
   */
  fun setTopVelocitySetpoint(rps: Double): REVLibError

  fun setTopP(p: Double): REVLibError

  fun setTopI(i: Double): REVLibError

  fun setTopD(d: Double): REVLibError

  fun setTopFF(ff: Double): REVLibError

  fun stopTop()
}