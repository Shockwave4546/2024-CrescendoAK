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
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.dovershockwave.utils.LoggedTunableBoolean
import org.dovershockwave.utils.LoggedTunableNumber
import org.littletonrobotics.junction.Logger

class IntakeArmSubsystem(private val intakeArm: IntakeArmIO) : SubsystemBase() {
  private val inputs = IntakeArmIO.IntakeArmIOInputs()
  private var desiredState = ArmState.HOME

  private val key = "IntakeArm"
  private val p = LoggedTunableNumber("$key/1.P", IntakeArmConstants.GAINS.p)
  private val i = LoggedTunableNumber("$key/2.I", IntakeArmConstants.GAINS.i)
  private val d = LoggedTunableNumber("$key/3.D", IntakeArmConstants.GAINS.d)
  private val ff = LoggedTunableNumber("$key/4.FF", IntakeArmConstants.GAINS.ff)

  private val useManualAngle = LoggedTunableBoolean("$key/5.UseManualAngle", false)
  private val manualAngle = LoggedTunableNumber("$key/6.ManualAngle", ArmState.HOME.angle)

  private val angleOffset = LoggedTunableNumber("$key/7.AngleOffset", IntakeArmConstants.ANGLE_OFFSET)

  override fun periodic() {
    intakeArm.updateInputs(inputs)
    Logger.processInputs(key, inputs)

    LoggedTunableNumber.ifChanged(hashCode(), { values ->
      intakeArm.setP(values[0])
      intakeArm.setI(values[1])
      intakeArm.setD(values[2])
      intakeArm.setFF(values[3])
    }, p, i, d, ff)

    LoggedTunableNumber.ifChanged(manualAngle.hashCode(), { values ->
      if (!useManualAngle.get()) return@ifChanged
      val clamped = MathUtil.clamp(values[0], IntakeArmConstants.MIN_ANGLE, IntakeArmConstants.MAX_ANGLE)
      intakeArm.setAngleSetpoint(clamped)
      this.desiredState = ArmState("Manual", clamped, null)
    }, manualAngle)

    LoggedTunableNumber.ifChanged(angleOffset.hashCode(), { values ->
      intakeArm.setAngleOffset(values[0])
    }, angleOffset)

    if (shouldStopArm()) {
      DriverStation.reportError("The encoder is reporting an angle that will break the arm: " + inputs.angle, false)
    }

    Logger.recordOutput("$key/DesiredState/1.Name", desiredState.name)
    Logger.recordOutput("$key/DesiredState/2.Angle", desiredState.angle)
    Logger.recordOutput("$key/DesiredState/3.At Goal", atDesiredState())
    Logger.recordOutput("$key/Should Stop Arm", shouldStopArm())
  }

  fun setDesiredState(desiredState: ArmState) {
    if (useManualAngle.get()) return
    if (shouldStopArm()) return

    val clamped = MathUtil.clamp(desiredState.angle, IntakeArmConstants.MIN_ANGLE, IntakeArmConstants.MAX_ANGLE)
    intakeArm.setAngleSetpoint(clamped)
    this.desiredState = desiredState
  }

  fun atDesiredState() = atDesiredState(desiredState)

  fun atDesiredState(state: ArmState) = inputs.angle in state.angle - IntakeArmConstants.ANGLE_TOLERANCE ..state.angle + IntakeArmConstants.ANGLE_TOLERANCE

  /**
   * If the Encoder is reading an angle that causes the arm to go into the robot, it should stop.
   * These angles include [198, 360].
   *
   * @return whether the arm should stop operating as to not break it.
   */
  private fun shouldStopArm() = inputs.angle > IntakeArmConstants.MAX_ANGLE
}