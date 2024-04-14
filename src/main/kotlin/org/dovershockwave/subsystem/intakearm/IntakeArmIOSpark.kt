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

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import org.dovershockwave.MotorConstants
import org.dovershockwave.utils.AbsSparkAction
import org.dovershockwave.utils.SparkUtils.Companion.configureAbs

class IntakeArmIOSpark(id: Int) : IntakeArmIO {
  private val motor = CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless)
  private val encoder = motor.absoluteEncoder
  private val pid = motor.pidController

  init {
    motor.configureAbs(linkedSetOf(
      AbsSparkAction("$id Set Smart Current Limit") { motor, _, _ -> motor.setSmartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT) },
      AbsSparkAction("$id Set Idle Mode") { motor, _, _ -> motor.setIdleMode(CANSparkBase.IdleMode.kBrake) },
      AbsSparkAction("$id Set Position Conversion Factor") { _, encoder, _ -> IntakeArmConstants.ANGLE_CONVERSION_FACTOR.apply(encoder) },
      AbsSparkAction("$id Set Zero Offset") { _, encoder, _ -> encoder.setZeroOffset(IntakeArmConstants.ANGLE_OFFSET) },
      AbsSparkAction("$id Set P Gain") { _, _, pid -> pid.setP(IntakeArmConstants.GAINS.p) },
      AbsSparkAction("$id Set I Gain") { _, _, pid -> pid.setI(IntakeArmConstants.GAINS.i) },
      AbsSparkAction("$id Set D Gain") { _, _, pid -> pid.setD(IntakeArmConstants.GAINS.d) },
      AbsSparkAction("$id Set Output Range") { _, _, pid -> pid.setFF(IntakeArmConstants.GAINS.ff) },
      AbsSparkAction("$id Set Output Range") { _, _, pid -> pid.setOutputRange(IntakeArmConstants.MIN_OUTPUT, IntakeArmConstants.MAX_OUTPUT) },
      AbsSparkAction("$id Set Feedback Device") { _, encoder, pid -> pid.setFeedbackDevice(encoder) },
      AbsSparkAction("Set Encoder Inverted") { _, encoder, _ -> encoder.setInverted(IntakeArmConstants.ENCODER_INVERTED) },
    ))
  }

  override fun updateInputs(inputs: IntakeArmIO.IntakeArmIOInputs) {
    inputs.angle = encoder.position
    inputs.appliedVolts = motor.busVoltage
    inputs.current = motor.outputCurrent
    inputs.temp = motor.motorTemperature
  }

  override fun setAngleSetpoint(angle: Double) {
    pid.setReference(angle, CANSparkBase.ControlType.kPosition)
  }
}