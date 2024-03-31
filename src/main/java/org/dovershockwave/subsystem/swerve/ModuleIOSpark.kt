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

package org.dovershockwave.subsystem.swerve

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.dovershockwave.MotorConstants
import org.dovershockwave.utils.configureAbs
import org.dovershockwave.utils.runBlockingRel

class ModuleIOSpark(driveID: Int, rotID: Int, private val chassisAngularOffset: Double) : ModuleIO {
  private val drivingMotor = CANSparkMax(driveID, CANSparkLowLevel.MotorType.kBrushless)
  private val driveEncoder = drivingMotor.encoder
  private val drivePID = drivingMotor.pidController

  private val turnMotor = CANSparkMax(rotID, CANSparkLowLevel.MotorType.kBrushless)
  private val turnEncoder = turnMotor.absoluteEncoder
  private val turnPID = turnMotor.pidController

  private var desiredState = SwerveModuleState(0.0, Rotation2d())

  init {
    drivingMotor.configureAbs { spark, encoder, pid ->
      spark.inverted = SwerveConstants.INVERT_DRIVING_DIRECTION
      pid.setFeedbackDevice(encoder)

      ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR.apply(encoder)
      encoder.setVelocityConversionFactor(ModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR)

      pid.p = ModuleConstants.DRIVING_GAINS.p.toDouble()
      pid.i = ModuleConstants.DRIVING_GAINS.i.toDouble()
      pid.d = ModuleConstants.DRIVING_GAINS.d.toDouble()
      pid.ff = ModuleConstants.DRIVING_GAINS.ff.toDouble()
      pid.setOutputRange(ModuleConstants.DRIVING_MIN_OUTPUT, ModuleConstants.DRIVING_MAX_OUTPUT)

      spark.idleMode = CANSparkBase.IdleMode.kBrake
      spark.setSmartCurrentLimit(MotorConstants.NEO_CURRENT_LIMIT)
      spark.enableVoltageCompensation(12.0)
    }

    turnMotor.configureAbs { spark, encoder, pid ->
      pid.setFeedbackDevice(encoder)

      ModuleConstants.TURNING_ENCODER_POSITION_FACTOR.apply(encoder)
      encoder.setVelocityConversionFactor(ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR)

      pid.setPositionPIDWrappingEnabled(true)
      pid.setPositionPIDWrappingMinInput(ModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT)
      pid.setPositionPIDWrappingMaxInput(ModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT)

      pid.p = ModuleConstants.TURNING_GAINS.p.toDouble()
      pid.i = ModuleConstants.TURNING_GAINS.i.toDouble()
      pid.d = ModuleConstants.TURNING_GAINS.d.toDouble()
      pid.ff = ModuleConstants.TURNING_GAINS.ff.toDouble()
      pid.setOutputRange(ModuleConstants.TURNING_MIN_OUTPUT, ModuleConstants.TURNING_MAX_OUTPUT)

      spark.idleMode = CANSparkBase.IdleMode.kBrake
      spark.setSmartCurrentLimit(MotorConstants.NEO_550_CURRENT_LIMIT)
    }

    desiredState.angle = Rotation2d(0.0) // TODO: verify this works. 
    resetDriveEncoder()
  }

  override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
    inputs.drivePosition = driveEncoder.position
    inputs.driveVelocity = driveEncoder.velocity
    inputs.driveAppliedVolts = drivingMotor.appliedOutput
    inputs.driveCurrent = drivingMotor.outputCurrent
    inputs.driveTemp = drivingMotor.motorTemperature

    inputs.rotPosition = Rotation2d(turnEncoder.position)
    inputs.rotVelocity = turnEncoder.velocity
    inputs.rotAppliedVolts = turnMotor.appliedOutput
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

  override fun getPosition() = SwerveModulePosition(driveEncoder.position, Rotation2d(turnEncoder.getPosition() - chassisAngularOffset));

  override fun setDesiredState(desiredState: SwerveModuleState) {
    // Apply chassis angular offset to the desired state.
    val correctedDesiredState = SwerveModuleState(
      desiredState.speedMetersPerSecond,
      desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset))
    )

    // Optimize the reference state to avoid spinning further than 90 degrees.
    val optimizedDesiredState =
      SwerveModuleState.optimize(correctedDesiredState, Rotation2d(turnEncoder.getPosition()))

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivePID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity)
    turnPID.setReference(optimizedDesiredState.angle.radians, CANSparkBase.ControlType.kPosition)

    this.desiredState = desiredState
  }

  override fun resetDriveEncoder() {
    drivingMotor.runBlockingRel { _, encoder, _ -> encoder.setPosition(0.0) }
  }
}