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

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.dovershockwave.utils.PolynomialRegression
import org.littletonrobotics.junction.Logger

class ShooterWristSubsystem(private val shooter: ShooterWristIO) : SubsystemBase() {
  private val inputs = ShooterWristIO.ShooterWristIOInputs()
  private var desiredState = WristState.STARTING

  private val anglePredictor = PolynomialRegression(
    doubleArrayOf(1.4, 1.6, 1.8, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2, 3.4, 4.0),
    doubleArrayOf(30.0, 30.0, 32.0, 32.0, 34.0, 40.0, 42.0, 45.0, 47.0, 47.0, 47.0, 47.0, 47.0),
    3,
    "x"
  )

  override fun periodic() {
    shooter.updateInputs(inputs)

    Logger.processInputs("Shooter Wrist", inputs)
    Logger.recordOutput("Shooter Wrist State Name", desiredState.name)
    Logger.recordOutput("Shooter Wrist State Angle", desiredState.angle)
  }
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