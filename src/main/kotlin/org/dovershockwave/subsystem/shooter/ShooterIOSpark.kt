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

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.REVLibError
import org.dovershockwave.MotorConstants
import org.dovershockwave.utils.RelSparkAction
import org.dovershockwave.utils.SparkUtils.Companion.configureRel
import org.dovershockwave.utils.SparkUtils.Companion.runBlockingRel

class ShooterIOSpark(bottomID: Int, topID: Int) : ShooterIO {
  private val bottomMotor = CANSparkMax(bottomID, CANSparkLowLevel.MotorType.kBrushless)
  private val bottomEncoder = bottomMotor.encoder
  private val bottomPID = bottomMotor.pidController

  private val topMotor = CANSparkMax(topID, CANSparkLowLevel.MotorType.kBrushless)
  private val topEncoder = topMotor.encoder
  private val topPID = topMotor.pidController

  init {
    bottomMotor.configureRel(linkedSetOf(
      RelSparkAction("Set Smart Current Limit") { spark, _, _ -> spark.setSmartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT) },
      RelSparkAction("Set Inverted") { spark, _, _ -> spark.inverted = ShooterConstants.BOT_INVERTED; REVLibError.kOk },
      RelSparkAction("Set Idle Mode") { spark, _, _ -> spark.setIdleMode(CANSparkBase.IdleMode.kBrake) },
      RelSparkAction("Set Position Conversion Factor") { _, encoder, _ -> ShooterConstants.REV_CONVERSION_FACTOR.apply(encoder) },
      RelSparkAction("Set Velocity Conversion Factor") { _, encoder, _ -> encoder.setVelocityConversionFactor(ShooterConstants.RPS_CONVERSION_FACTOR) },
      RelSparkAction("Set PID P") { _, _, pid -> pid.setP(ShooterConstants.BOT_GAINS.p) },
      RelSparkAction("Set PID I") { _, _, pid -> pid.setI(ShooterConstants.BOT_GAINS.i) },
      RelSparkAction("Set PID D") { _, _, pid -> pid.setD(ShooterConstants.BOT_GAINS.d) },
      RelSparkAction("Set PID FF") { _, _, pid -> pid.setFF(ShooterConstants.BOT_GAINS.ff) },
      RelSparkAction("Set PID Output Range") { _, _, pid -> pid.setOutputRange(ShooterConstants.MIN_OUTPUT, ShooterConstants.MAX_OUTPUT) },
      RelSparkAction("Set Feedback Device") { _, encoder, pid -> pid.setFeedbackDevice(encoder) }
    ))

    topMotor.configureRel(linkedSetOf(
      RelSparkAction("Set Smart Current Limit") { spark, _, _ -> spark.setSmartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT) },
      RelSparkAction("Set Inverted") { spark, _, _ -> spark.inverted = ShooterConstants.TOP_INVERTED; REVLibError.kOk },
      RelSparkAction("Set Idle Mode") { spark, _, _ -> spark.setIdleMode(CANSparkBase.IdleMode.kBrake) },
      RelSparkAction("Set Position Conversion Factor") { _, encoder, _ -> ShooterConstants.REV_CONVERSION_FACTOR.apply(encoder) },
      RelSparkAction("Set Velocity Conversion Factor") { _, encoder, _ -> encoder.setVelocityConversionFactor(ShooterConstants.RPS_CONVERSION_FACTOR) },
      RelSparkAction("Set PID P") { _, _, pid -> pid.setP(ShooterConstants.TOP_GAINS.p) },
      RelSparkAction("Set PID I") { _, _, pid -> pid.setI(ShooterConstants.TOP_GAINS.i) },
      RelSparkAction("Set PID D") { _, _, pid -> pid.setD(ShooterConstants.TOP_GAINS.d) },
      RelSparkAction("Set PID FF") { _, _, pid -> pid.setFF(ShooterConstants.TOP_GAINS.ff) },
      RelSparkAction("Set PID Output Range") { _, _, pid -> pid.setOutputRange(ShooterConstants.MIN_OUTPUT, ShooterConstants.MAX_OUTPUT) },
      RelSparkAction("Set Feedback Device") { _, encoder, pid -> pid.setFeedbackDevice(encoder) }
    ))
  }

  override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
    inputs.bottomRPS = bottomEncoder.velocity
    inputs.bottomAppliedVolts = bottomMotor.appliedOutput * bottomMotor.busVoltage
    inputs.bottomCurrent = bottomMotor.outputCurrent
    inputs.bottomTemp = bottomMotor.motorTemperature

    inputs.topRPS = topEncoder.velocity
    inputs.topAppliedVolts = topMotor.appliedOutput * topMotor.busVoltage
    inputs.topCurrent = topMotor.outputCurrent
    inputs.topTemp = topMotor.motorTemperature
  }

  override fun setBottomVelocitySetpoint(rps: Double) {
    bottomMotor.runBlockingRel(linkedSetOf(
      RelSparkAction("a") { _, _, pid -> pid.setReference(rps, CANSparkBase.ControlType.kVelocity)}
    ))
  }

  override fun setBotP(p: Double) {
    bottomPID.setP(p)
  }

  override fun setBotI(i: Double) {
    bottomPID.setI(i)
  }

  override fun setBotD(d: Double) {
    bottomPID.setD(d)
  }

  override fun setBotFF(ff: Double) {
    bottomPID.setFF(ff)
  }

  override fun setTopVelocitySetpoint(rps: Double) {
    topMotor.runBlockingRel(linkedSetOf(
      RelSparkAction("b") { _, _, pid -> pid.setReference(rps, CANSparkBase.ControlType.kVelocity)}
    ))
  }

  override fun setTopP(p: Double) {
    topPID.setP(p)
  }

  override fun setTopI(i: Double) {
    topPID.setI(i)
  }

  override fun setTopD(d: Double) {
    topPID.setD(d)
  }

  override fun setTopFF(ff: Double) {
    topPID.setFF(ff)
  }
}