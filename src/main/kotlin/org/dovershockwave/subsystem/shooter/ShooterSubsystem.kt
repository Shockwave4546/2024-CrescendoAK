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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.dovershockwave.shuffleboard.TunableSparkPIDController
import org.dovershockwave.subsystem.pose.PoseEstimatorSubsystem
import org.dovershockwave.utils.PolynomialRegression
import org.littletonrobotics.junction.Logger

class ShooterSubsystem(private val shooter: ShooterIO, private val poseEstimator: PoseEstimatorSubsystem) : SubsystemBase() {
  private val inputs = ShooterIO.ShooterIOInputs()
  private var desiredState = ShooterState.STOPPED

  private val bottomRPSPredictor = PolynomialRegression(
    doubleArrayOf(1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2, 3.4, 4.0),
    doubleArrayOf(60.0, 60.0, 60.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0),
    3,
    "x"
  )

  private val topRPSPredictor = PolynomialRegression(
    doubleArrayOf(1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2, 3.4, 4.0),
    doubleArrayOf(35.0, 35.0, 40.0, 55.0, 63.0, 65.0, 65.0, 65.0, 65.0, 65.0, 65.0, 65.0),
    3,
    "x"
  )

  val tab = Shuffleboard.getTab("Shooter")

  init {
    tab.add("Top PID",
      TunableSparkPIDController(shooter.getRawTop(), { desiredState.topRPS }, { topRPS: Double ->
        this.desiredState = ShooterState("Manual", desiredState.bottomRPS, topRPS)
        shooter.setTopVelocitySetpoint(topRPS)
      })
    )

    tab.add("Bottom PID",
      TunableSparkPIDController(shooter.getRawBot(), { desiredState.bottomRPS }, { botRPS: Double ->
        this.desiredState = ShooterState("Manual", botRPS, desiredState.topRPS)
        shooter.setBottomVelocitySetpoint(botRPS)
      })
    )
  }

  override fun periodic() {
    shooter.updateInputs(inputs)

    val key = "Shooter"
    Logger.processInputs(key, inputs)
    Logger.recordOutput("$key/Desired State", desiredState.name)
    Logger.recordOutput("$key/Desired State Bot RPS", desiredState.bottomRPS)
    Logger.recordOutput("$key/Desired State Top RPS", desiredState.topRPS)
    Logger.recordOutput("$key/At Desired State", atDesiredState())
  }

  fun setDesiredState(state: ShooterState) {
    if (state === ShooterState.INTERPOLATED) {
      val distance = poseEstimator.getToSpeakerFromVision().distance
      if (!distance.isPresent) return
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