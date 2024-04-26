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

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.dovershockwave.shuffleboard.TunableSparkPIDController
import org.dovershockwave.subsystem.vision.VisionSubsystem
import org.dovershockwave.utils.PolynomialRegression
import org.littletonrobotics.junction.Logger

class ShooterWristSubsystem(private val wrist: ShooterWristIO, private val poseEstimator: VisionSubsystem) : SubsystemBase() {
  private val inputs = ShooterWristIO.ShooterWristIOInputs()
  private var desiredState = WristState.STARTING

  private val anglePredictor = PolynomialRegression(
    doubleArrayOf(1.4, 1.6, 1.8, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2, 3.4, 4.0),
    doubleArrayOf(30.0, 30.0, 32.0, 32.0, 34.0, 40.0, 42.0, 45.0, 47.0, 47.0, 47.0, 47.0, 47.0),
    3,
    "x"
  )

  init {
    val tab = Shuffleboard.getTab("ShooterWrist")
    tab.add("Top PID",
      TunableSparkPIDController(wrist.pid(), { desiredState.angle }, { angle: Double ->
        this.desiredState = WristState("Manual", angle)
        wrist.setAngleSetpoint(angle)
      })
    )
  }

  override fun periodic() {
    wrist.updateInputs(inputs)

    if (shouldStopWrist()) {
      DriverStation.reportError("The encoder is reporting an angle that will break the wrist: " + inputs.angle, false)
    }

    val key = "Shooter Wrist"
    Logger.processInputs(key, inputs)
    Logger.recordOutput("$key/State Name", desiredState.name)
    Logger.recordOutput("$key/State Angle", desiredState.angle)
    Logger.recordOutput("$key/At Desired State", atDesiredState())
    Logger.recordOutput("$key/Should Stop Wrist", shouldStopWrist())
  }

  fun setDesiredState(state: WristState) {
    if (shouldStopWrist()) return
    if (state === WristState.INTERPOLATED) {
      val distance = poseEstimator.getToSpeakerFromVision().distance
      if (distance.isEmpty) return
      this.desiredState = WristState("Interpolated", anglePredictor.predict(distance.get()))
    } else {
      this.desiredState = state
    }

    val clamped = MathUtil.clamp(desiredState.angle, WristConstants.MIN_ANGLE, WristConstants.MAX_ANGLE)
    wrist.setAngleSetpoint(clamped)
  }

  fun atDesiredState() = inputs.angle in desiredState.angle - WristConstants.ANGLE_TOLERANCE ..desiredState.angle + WristConstants.ANGLE_TOLERANCE

  /**
   * If the Encoder is reading an angle that causes the wrist to go into the robot, it should stop.
   * These angles include [81, 360].
   *
   * @return whether the wrist should stop operating as to not break it.
   */
  private fun shouldStopWrist() = inputs.angle > WristConstants.MAX_ANGLE
}

fun main() {
  val anglePredictor = PolynomialRegression(
    doubleArrayOf(1.4, 1.6, 1.8, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2, 3.4, 4.0),
    doubleArrayOf(30.0, 30.0, 32.0, 32.0, 34.0, 40.0, 42.0, 45.0, 47.0, 47.0, 47.0, 47.0, 47.0),
    3,
    "x"
  )

  println(anglePredictor)
}