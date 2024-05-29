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

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.DigitalInput
import org.dovershockwave.MotorConstants
import org.dovershockwave.utils.RelSparkAction
import org.dovershockwave.utils.SparkUtils.Companion.configureRel

class IntakeIOSpark(id: Int) : IntakeIO {
  private val motor = CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless)
  private val limitSwitch = DigitalInput(IntakeConstants.LIMIT_SWITCH_DIO_PORT)

  init {
    motor.configureRel(linkedSetOf(
      RelSparkAction("Set Smart Current Limit") { motor, _, _ -> motor.setSmartCurrentLimit(MotorConstants.NEO_550_CURRENT_LIMIT) },
      RelSparkAction("Set Idle Mode") { motor, _, _ -> motor.setIdleMode(CANSparkBase.IdleMode.kBrake) }
    ))
  }

  override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
    inputs.dutyCycle = motor.appliedOutput
    inputs.appliedVolts = motor.appliedOutput * motor.busVoltage
    inputs.current = motor.outputCurrent
    inputs.temp = motor.motorTemperature
    inputs.hasNote = hasNote()
  }

  override fun setDutyCycle(dutyCycle: Double) {
    motor.set(dutyCycle)
  }

  override fun hasNote() = limitSwitch.get()
}