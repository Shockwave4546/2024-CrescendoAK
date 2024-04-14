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

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.dovershockwave.shuffleboard.TunableSparkPIDController
import org.dovershockwave.subsystem.shooterwrist.WristState
import org.littletonrobotics.junction.Logger

class IntakeArmSubsystem(private val intakeArm: IntakeArmIO) : SubsystemBase() {
  private val inputs = IntakeArmIO.IntakeArmIOInputs()
  private var desiredState = ArmState.HOME

  init {
    val tab = Shuffleboard.getTab("IntakeArm")
    tab.add("PID",
      TunableSparkPIDController(intakeArm.pid(), { desiredState.angle }, { angle: Double ->
        this.desiredState = ArmState("Manual", angle)
        intakeArm.setAngleSetpoint(angle)
      })
    )
  }

  override fun periodic() {
    intakeArm.updateInputs(inputs)

    if (shouldStopArm()) {
      DriverStation.reportError("The encoder is reporting an angle that will break the arm: " + inputs.angle, false)
    }


    val key = "Intake Arm"
    Logger.processInputs(key, inputs)
    Logger.recordOutput("$key/State Name", desiredState.name)
    Logger.recordOutput("$key/State Angle", desiredState.angle)
    Logger.recordOutput("$key/At Desired State", atDesiredState())
    Logger.recordOutput("$key/Should Stop Arm", shouldStopArm())
  }

  fun setDesiredState(desiredState: ArmState) {
    if (shouldStopArm()) return

    val clamped = MathUtil.clamp(desiredState.angle, IntakeArmConstants.MIN_ANGLE, IntakeArmConstants.MAX_ANGLE)
    intakeArm.setAngleSetpoint(clamped)
  }

  fun atDesiredState() = inputs.angle in desiredState.angle - IntakeArmConstants.ANGLE_TOLERANCE ..desiredState.angle + IntakeArmConstants.ANGLE_TOLERANCE

  /**
   * If the Encoder is reading an angle that causes the arm to go into the robot, it should stop.
   * These angles include [198, 360].
   *
   * @return whether the arm should stop operating as to not break it.
   */
  private fun shouldStopArm() = inputs.angle > IntakeArmConstants.MAX_ANGLE
}