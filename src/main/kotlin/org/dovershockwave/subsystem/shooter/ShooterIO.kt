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

import com.revrobotics.SparkPIDController
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
      table.put("Bottom RPS", bottomRPS)
      table.put("Bottom Applied Volts", bottomAppliedVolts)
      table.put("Bottom Current", bottomCurrent)
      table.put("Bottom Temp", bottomTemp)

      table.put("Top RPS", topRPS)
      table.put("Top Applied Volts", topAppliedVolts)
      table.put("Top Current", topCurrent)
      table.put("Top Temp", topTemp)
    }

    override fun fromLog(table: LogTable) {
      bottomRPS = table.get("Bottom RPS", bottomRPS)
      bottomAppliedVolts = table.get("Bottom Applied Volts", bottomAppliedVolts)
      bottomCurrent = table.get("Bottom Current", bottomCurrent)
      bottomTemp = table.get("Bottom Temp", bottomTemp)

      topRPS = table.get("Top RPS", topRPS)
      topAppliedVolts = table.get("Top Applied Volts", topAppliedVolts)
      topCurrent = table.get("Top Current", topCurrent)
      topTemp = table.get("Top Temp", topTemp)
    }
  }

  fun updateInputs(inputs: ShooterIOInputs)

  fun setBottomVelocitySetpoint(rps: Double)

  fun setTopVelocitySetpoint(rps: Double)

  fun getRawBot(): SparkPIDController

  fun getRawTop(): SparkPIDController
}