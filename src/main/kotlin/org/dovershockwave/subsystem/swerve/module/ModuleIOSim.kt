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

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.DCMotorSim

class ModuleIOSim(private val chassisAngularOffset: Double) : ModuleIO {
  private val driveSim =
    DCMotorSim(DCMotor.getNEO(1), ModuleConstants.DRIVING_MOTOR_REDUCTION, 0.025) // idk the jKhMeterSquared
  private val drivePID = PIDController(
    ModuleConstants.DRIVING_GAINS.p.toDouble(),
    ModuleConstants.DRIVING_GAINS.i.toDouble(),
    ModuleConstants.DRIVING_GAINS.d.toDouble()
  )
  private var driveAppliedVoltage = 0.0

  private val turnSim = DCMotorSim(DCMotor.getNeo550(1), 1.0, 0.004) // idk the jKhMeterSquared or the reduction
  private val turnPID = PIDController(
    ModuleConstants.TURNING_GAINS.p.toDouble(),
    ModuleConstants.TURNING_GAINS.i.toDouble(),
    ModuleConstants.TURNING_GAINS.d.toDouble()
  )
  private var turnAppliedVoltage = 0.0

  private var desiredState = SwerveModuleState(0.0, Rotation2d())

  override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
    inputs.drivePosition = getLinearPosition()
    inputs.driveVelocity = getLinearVelocity()
    inputs.driveAppliedVolts = driveAppliedVoltage
    inputs.driveCurrent = driveSim.currentDrawAmps
    inputs.driveTemp = 0.0 // Temperature doesn't exist for simulation.

    inputs.rotPosition = Rotation2d(turnSim.angularPositionRad)
    inputs.rotVelocity = turnSim.angularVelocityRadPerSec
    inputs.rotAppliedVolts = turnAppliedVoltage
    inputs.rotCurrent = turnSim.currentDrawAmps
    inputs.rotTemp = 0.0 // Temperature doesn't exist for simulation.
  }

  private fun getLinearPosition() = driveSim.angularPositionRotations * ModuleConstants.WHEEL_DIAMETER_METERS

  private fun getLinearVelocity() = driveSim.angularVelocityRPM * (ModuleConstants.WHEEL_DIAMETER_METERS / 60.0)

  override fun getState() =
    SwerveModuleState(getLinearVelocity(), Rotation2d(turnSim.angularPositionRad - chassisAngularOffset))

  override fun getPosition() =
    SwerveModulePosition(getLinearPosition(), Rotation2d(turnSim.angularPositionRad - chassisAngularOffset))

  override fun setDesiredState(desiredState: SwerveModuleState) {
    // Apply chassis angular offset to the desired state.
    val correctedDesiredState = SwerveModuleState(
      desiredState.speedMetersPerSecond,
      desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset))
    )

    // Optimize the reference state to avoid spinning further than 90 degrees.
    val optimizedDesiredState =
      SwerveModuleState.optimize(correctedDesiredState, Rotation2d(turnSim.angularPositionRad))

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivePID.setpoint = optimizedDesiredState.speedMetersPerSecond
    turnPID.setpoint = optimizedDesiredState.angle.radians

    this.desiredState = desiredState
  }

  override fun resetDriveEncoder() = driveSim.setState(0.0, 0.0)
}