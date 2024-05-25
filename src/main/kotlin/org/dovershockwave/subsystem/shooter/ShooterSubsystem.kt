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

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.dovershockwave.subsystem.vision.VisionSubsystem
import org.dovershockwave.utils.LoggedTunableBoolean
import org.dovershockwave.utils.LoggedTunableNumber
import org.dovershockwave.utils.PolynomialRegression
import org.littletonrobotics.junction.Logger

class ShooterSubsystem(private val shooter: ShooterIO, private val vision: VisionSubsystem) : SubsystemBase() {
  private val inputs = ShooterIO.ShooterIOInputs()
  private var desiredState = ShooterState.STOPPED

  // DCMP
//  private val bottomRPSPredictor = PolynomialRegression(
//    doubleArrayOf(1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2, 3.4, 4.0),
//    doubleArrayOf(60.0, 60.0, 60.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0),
//    3,
//    "x"
//  )
//
//  private val topRPSPredictor = PolynomialRegression(
//    doubleArrayOf(1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2, 3.4, 4.0),
//    doubleArrayOf(35.0, 35.0, 40.0, 55.0, 63.0, 65.0, 65.0, 65.0, 65.0, 65.0, 65.0, 65.0),
//    3,
//    "x"
//  )

  private val bottomRPSPredictor = PolynomialRegression(
    doubleArrayOf(1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2),
    doubleArrayOf(60.0, 60.0, 60.0, 40.0, 40.0, 40.0, 45.0, 50.0, 60.0, 60.0),
    7,
    "x"
  )

  private val topRPSPredictor = PolynomialRegression(
    doubleArrayOf(1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2),
    doubleArrayOf(30.0, 30.0, 45.0, 50.0, 60.0, 55.0, 60.0, 60.0, 70.0, 75.0),
    3,
    "x"
  )

  private val key = "Shooter"
  private val useManualSpeed = LoggedTunableBoolean("$key/1.UseManualSpeed", false)

  private val botP = LoggedTunableNumber("$key/Bot/1.P", ShooterConstants.BOT_GAINS.p)
  private val botI = LoggedTunableNumber("$key/Bot/2.I", ShooterConstants.BOT_GAINS.i)
  private val botD = LoggedTunableNumber("$key/Bot/3.D", ShooterConstants.BOT_GAINS.d)
  private val botFF = LoggedTunableNumber("$key/Bot/4.FF", ShooterConstants.BOT_GAINS.ff)
  private val botManualSpeed = LoggedTunableNumber("$key/Bot/5.ManualSpeed", 0.0)

  private val topP = LoggedTunableNumber("$key/Top/1.P", ShooterConstants.TOP_GAINS.p)
  private val topI = LoggedTunableNumber("$key/Top/2.I", ShooterConstants.TOP_GAINS.i)
  private val topD = LoggedTunableNumber("$key/Top/3.D", ShooterConstants.TOP_GAINS.d)
  private val topFF = LoggedTunableNumber("$key/Top/4.FF", ShooterConstants.TOP_GAINS.ff)
  private val topManualSpeed = LoggedTunableNumber("$key/Top/5.ManualSpeed", 0.0)

  override fun periodic() {
    shooter.updateInputs(inputs)
    Logger.processInputs(key, inputs)

    LoggedTunableNumber.ifChanged(hashCode(), { values ->
      shooter.setBotP(values[0])
      shooter.setBotI(values[1])
      shooter.setBotD(values[2])
      shooter.setBotFF(values[3])
    }, botP, botI, botD, botFF)

    LoggedTunableNumber.ifChanged(hashCode(), { values ->
      shooter.setTopP(values[0])
      shooter.setTopI(values[1])
      shooter.setTopD(values[2])
      shooter.setTopFF(values[3])
    }, topP, topI, topD, topFF)

    LoggedTunableNumber.ifChanged(botManualSpeed.hashCode() + topManualSpeed.hashCode(), { values ->
      if (!useManualSpeed.get()) return@ifChanged
      val botClamped = MathUtil.clamp(values[0], ShooterConstants.MIN_RPS, ShooterConstants.MAX_RPS)
      val topClamped = MathUtil.clamp(values[1], ShooterConstants.MIN_RPS, ShooterConstants.MAX_RPS)
      shooter.setBottomVelocitySetpoint(botClamped)
      shooter.setTopVelocitySetpoint(topClamped)
    }, botManualSpeed, topManualSpeed)

    Logger.recordOutput("$key/DesiredState/1.Name", desiredState.name)
    Logger.recordOutput("$key/DesiredState/2.Bot RPS", desiredState.bottomRPS)
    Logger.recordOutput("$key/DesiredState/3.Top RPS", desiredState.topRPS)
    Logger.recordOutput("$key/DesiredState/4.At Goal", atDesiredState())
  }

  fun setDesiredState(state: ShooterState) {
    if (useManualSpeed.get()) return
    if (state === ShooterState.INTERPOLATED) {
      val distance = vision.getToSpeakerFromVision().distance
      if (distance.isEmpty) return
      this.desiredState = ShooterState("Interpolated", bottomRPSPredictor.predict(distance.get()), topRPSPredictor.predict(distance.get()))
    } else {
      this.desiredState = state
    }

    val bottomClamped = MathUtil.clamp(desiredState.bottomRPS, ShooterConstants.MIN_RPS, ShooterConstants.MAX_RPS)
    val topClamped = MathUtil.clamp(desiredState.topRPS, ShooterConstants.MIN_RPS, ShooterConstants.MAX_RPS)
    shooter.setBottomVelocitySetpoint(bottomClamped)
    shooter.setTopVelocitySetpoint(topClamped)
  }

  fun atDesiredState() =
    inputs.bottomRPS in desiredState.bottomRPS - ShooterConstants.RPS_TOLERANCE ..desiredState.bottomRPS + ShooterConstants.RPS_TOLERANCE &&
            inputs.topRPS in desiredState.topRPS - ShooterConstants.RPS_TOLERANCE ..desiredState.topRPS + ShooterConstants.RPS_TOLERANCE
}

fun main() {
  val bottomRPSPredictor = PolynomialRegression(
    doubleArrayOf(1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2),
    doubleArrayOf(60.0, 60.0, 60.0, 40.0, 40.0, 40.0, 45.0, 50.0, 60.0, 60.0),
    7,
    "x"
  )

  val topRPSPredictor = PolynomialRegression(
    doubleArrayOf(1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2),
    doubleArrayOf(30.0, 30.0, 45.0, 50.0, 60.0, 55.0, 60.0, 60.0, 70.0, 75.0),
    3,
    "x"
  )

  println("bottomRPSPredictor = ${bottomRPSPredictor}")

  println("topRPSPredictor = ${topRPSPredictor}")
}