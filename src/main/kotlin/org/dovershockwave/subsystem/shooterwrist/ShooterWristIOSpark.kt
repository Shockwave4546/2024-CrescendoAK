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

package org.dovershockwave.subsystem.shooterwrist

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import org.dovershockwave.MotorConstants
import org.dovershockwave.utils.AbsSparkAction
import org.dovershockwave.utils.SparkUtils.Companion.configureAbs

class ShooterWristIOSpark(id: Int) : ShooterWristIO {
  private val motor = CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless)
  private val encoder = motor.absoluteEncoder
  private val pid = motor.pidController

  init {
    motor.configureAbs(linkedSetOf(
      AbsSparkAction("$id Set Smart Current Limit") { motor, _, _ -> motor.setSmartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT) },
      AbsSparkAction("$id Set Idle Mode") { motor, _, _ -> motor.setIdleMode(CANSparkBase.IdleMode.kBrake) },
      AbsSparkAction("$id Set Position Conversion Factor") { _, encoder, _ -> WristConstants.ANGLE_CONVERSION_FACTOR.apply(encoder) },
      AbsSparkAction("$id Set Encoder Inverted") { _, encoder, _ -> encoder.setInverted(WristConstants.ENCODER_INVERTED) },
      AbsSparkAction("$id Set Zero Offset") { _, encoder, _ -> encoder.setZeroOffset(WristConstants.ANGLE_OFFSET) },
      AbsSparkAction("$id Set P Gain") { _, _, pid -> pid.setP(WristConstants.GAINS.p) },
      AbsSparkAction("$id Set I Gain") { _, _, pid -> pid.setI(WristConstants.GAINS.i) },
      AbsSparkAction("$id Set D Gain") { _, _, pid -> pid.setD(WristConstants.GAINS.d) },
      AbsSparkAction("$id Set Output Range") { _, _, pid -> pid.setFF(WristConstants.GAINS.ff) },
      AbsSparkAction("$id Set Output Range") { _, _, pid -> pid.setOutputRange(WristConstants.MIN_OUTPUT, WristConstants.MAX_OUTPUT) },
      AbsSparkAction("$id Set Feedback Device") { _, encoder, pid -> pid.setFeedbackDevice(encoder) }
    ))
  }

  override fun updateInputs(inputs: ShooterWristIO.ShooterWristIOInputs) {
    inputs.angle = encoder.position
    inputs.appliedVolts = motor.busVoltage
    inputs.current = motor.outputCurrent
    inputs.temp = motor.motorTemperature
  }

  override fun setAngleSetpoint(angle: Double) {
    pid.setReference(angle, CANSparkBase.ControlType.kPosition)
  }

  override fun setP(p: Double) {
    pid.setP(p)
  }

  override fun setI(i: Double) {
    pid.setI(i)
  }

  override fun setD(d: Double) {
    pid.setD(d)
  }

  override fun setFF(ff: Double) {
    pid.setFF(ff)
  }
}