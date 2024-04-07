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

import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim

// TODO:  
class ModuleIOSim : ModuleIO {
  private val driveSim = DCMotorSim(DCMotor.getNEO(1), ModuleConstants.DRIVING_MOTOR_REDUCTION, 0.025) // idk the jKhMeterSquared
  private val turnSim = DCMotorSim(DCMotor.getNeo550(1), 1.0, 0.004) // idk the jKhMeterSquared or the reduction

  override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
    TODO("Not yet implemented")
  }

  override fun getState(): SwerveModuleState {
    TODO("Not yet implemented")
  }

  override fun getPosition(): SwerveModulePosition {
    TODO("Not yet implemented")
  }

  override fun setDesiredState(desiredState: SwerveModuleState) {
    TODO("Not yet implemented")
  }

  override fun resetDriveEncoder() {
    TODO("Not yet implemented")
  }
}