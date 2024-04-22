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

package org.dovershockwave.subsystem.swerve.module

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.REVLibError
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.dovershockwave.MotorConstants
import org.dovershockwave.subsystem.swerve.SwerveConstants
import org.dovershockwave.utils.AbsSparkAction
import org.dovershockwave.utils.RelSparkAction
import org.dovershockwave.utils.SparkUtils.Companion.configureAbs
import org.dovershockwave.utils.SparkUtils.Companion.configureRel
import org.dovershockwave.utils.SparkUtils.Companion.runBlockingRel
import java.util.UUID

class ModuleIOSpark(driveID: Int, rotID: Int, private val chassisAngularOffset: Double) : ModuleIO {
  private val drivingMotor = CANSparkMax(driveID, CANSparkLowLevel.MotorType.kBrushless)
  private val driveEncoder = drivingMotor.encoder
  private val drivePID = drivingMotor.pidController

  private val turnMotor = CANSparkMax(rotID, CANSparkLowLevel.MotorType.kBrushless)
  private val turnEncoder = turnMotor.absoluteEncoder
  private val turnPID = turnMotor.pidController

  private var desiredState = SwerveModuleState(0.0, Rotation2d())

  init {
    drivingMotor.configureRel(linkedSetOf(
      RelSparkAction("$driveID Set Drive Inverted") { spark, _, _ -> spark.inverted = SwerveConstants.INVERT_DRIVING_DIRECTION; REVLibError.kOk },
      RelSparkAction("$driveID Set Feedback Device") { _, encoder, pid -> pid.setFeedbackDevice(encoder) },
      RelSparkAction("$driveID Set Position Conversion Factor") { _, encoder, _ -> ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR.apply(encoder) },
      RelSparkAction("$driveID Set Velocity Conversion Factor") { _, encoder, _ -> encoder.setVelocityConversionFactor(ModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR) },
      RelSparkAction("$driveID Set P Gain") { _, _, pid -> pid.setP(ModuleConstants.DRIVING_GAINS.p) },
      RelSparkAction("$driveID Set I Gain") { _, _, pid -> pid.setI(ModuleConstants.DRIVING_GAINS.i) },
      RelSparkAction("$driveID Set D Gain") { _, _, pid -> pid.setD(ModuleConstants.DRIVING_GAINS.d) },
      RelSparkAction("$driveID Set FF Gain") { _, _, pid -> pid.setFF(ModuleConstants.DRIVING_GAINS.ff) },
      RelSparkAction("$driveID Set Output Range") { _, _, pid -> pid.setOutputRange(ModuleConstants.DRIVING_MIN_OUTPUT, ModuleConstants.DRIVING_MAX_OUTPUT) },
      RelSparkAction("$driveID Set Idle Mode") { spark, _, _ -> spark.setIdleMode(CANSparkBase.IdleMode.kBrake) },
      RelSparkAction("$driveID Set Smart Current Limit") { spark, _, _ -> spark.setSmartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT) },
      RelSparkAction("$driveID Enable Voltage Compensation") { spark, _, _ -> spark.enableVoltageCompensation(ModuleConstants.VOLTAGE_COMPENSATION) }
    ))

    turnMotor.configureAbs(linkedSetOf(
      AbsSparkAction("$rotID Set Turn Inverted") { _, encoder, _ -> encoder.setInverted(ModuleConstants.TURNING_ENCODER_INVERTED) },
      AbsSparkAction("$rotID Set Feedback Device") { _, encoder, pid -> pid.setFeedbackDevice(encoder) },
      AbsSparkAction("$rotID Set Position Conversion Factor") { _, encoder, _ -> ModuleConstants.TURNING_ENCODER_POSITION_FACTOR.apply(encoder) },
      AbsSparkAction("$rotID Set Velocity Conversion Factor") { _, encoder, _ -> encoder.setVelocityConversionFactor(ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR) },
      AbsSparkAction("$rotID Set P Gain") { _, _, pid -> pid.setP(ModuleConstants.TURNING_GAINS.p) },
      AbsSparkAction("$rotID Set I Gain") { _, _, pid -> pid.setI(ModuleConstants.TURNING_GAINS.i) },
      AbsSparkAction("$rotID Set D Gain") { _, _, pid -> pid.setD(ModuleConstants.TURNING_GAINS.d) },
      AbsSparkAction("$rotID Set FF Gain") { _, _, pid -> pid.setFF(ModuleConstants.TURNING_GAINS.ff) },
      AbsSparkAction("$rotID Set Output Range") { _, _, pid -> pid.setOutputRange(ModuleConstants.TURNING_MIN_OUTPUT, ModuleConstants.TURNING_MAX_OUTPUT) },
      AbsSparkAction("$rotID Set Idle Mode") { spark, _, _ -> spark.setIdleMode(CANSparkBase.IdleMode.kBrake) },
      AbsSparkAction("$rotID Set Smart Current Limit") { spark, _, _ -> spark.setSmartCurrentLimit(MotorConstants.NEO_550_CURRENT_LIMIT) },
      AbsSparkAction("$rotID Enable Voltage Compensation") { spark, _, _ -> spark.enableVoltageCompensation(ModuleConstants.VOLTAGE_COMPENSATION) },
      AbsSparkAction("$rotID Enable Wrapping") { _, _, pid -> pid.setPositionPIDWrappingEnabled(true) },
      AbsSparkAction("$rotID Set Wrapping Min") { _, _, pid -> pid.setPositionPIDWrappingMaxInput(ModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT) },
      AbsSparkAction("$rotID Set Wrapping Max") { _, _, pid -> pid.setPositionPIDWrappingMaxInput(ModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT) }
    ))

    desiredState.angle = Rotation2d(0.0)
    resetDriveEncoder()
  }

  override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
    inputs.drivePosition = driveEncoder.position
    inputs.driveVelocity = driveEncoder.velocity
    inputs.driveAppliedVolts = drivingMotor.busVoltage
    inputs.driveCurrent = drivingMotor.outputCurrent
    inputs.driveTemp = drivingMotor.motorTemperature

    inputs.rotPosition = Rotation2d(turnEncoder.position)
    inputs.rotVelocity = turnEncoder.velocity
    inputs.rotAppliedVolts = turnMotor.busVoltage
    inputs.rotCurrent = turnMotor.outputCurrent
    inputs.rotTemp = turnMotor.motorTemperature
  }

  /**
   * Returns the current state of the module. A chassis angular offset is applied to the encoder position
   * to get the position relative to the chassis.
   *
   * @return The current state of the module.
   */
  override fun getState() = SwerveModuleState(driveEncoder.velocity, Rotation2d(turnEncoder.getPosition() - chassisAngularOffset))

  override fun getPosition() = SwerveModulePosition(driveEncoder.position, Rotation2d(turnEncoder.getPosition() - chassisAngularOffset))

  override fun setDesiredState(desiredState: SwerveModuleState) {
    // Apply chassis angular offset to the desired state.
    val correctedDesiredState = SwerveModuleState(
      desiredState.speedMetersPerSecond,
      desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset))
    )

    // Optimize the reference state to avoid spinning further than 90 degrees.
    val optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, Rotation2d(turnEncoder.getPosition()))

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivePID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity)
    turnPID.setReference(optimizedDesiredState.angle.radians, CANSparkBase.ControlType.kPosition)

    this.desiredState = desiredState
  }

  override fun resetDriveEncoder() {
    drivingMotor.runBlockingRel(linkedSetOf(
      RelSparkAction("Reset Drive Encoder #${UUID.randomUUID()}") { _, encoder, _ -> encoder.setPosition(0.0) }
    ))
  }
}