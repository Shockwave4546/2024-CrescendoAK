package org.dovershockwave.subsystem.swerve.module

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.dovershockwave.utils.LoggedTunableNumber
import org.littletonrobotics.junction.Logger

class Module(private val module: ModuleIO, private val key: String) {
  private val inputs = ModuleIO.ModuleIOInputs()
  private var desiredState = SwerveModuleState(0.0, Rotation2d())

  private val driveP = LoggedTunableNumber("$key/Drive/1.P", ModuleConstants.DRIVING_GAINS.p)
  private val driveI = LoggedTunableNumber("$key/Drive/2.I", ModuleConstants.DRIVING_GAINS.i)
  private val driveD = LoggedTunableNumber("$key/Drive/3.D", ModuleConstants.DRIVING_GAINS.d)
  private val driveFF = LoggedTunableNumber("$key/Drive/4.FF", ModuleConstants.DRIVING_GAINS.ff)

  private val turnP = LoggedTunableNumber("$key/Turn/1.P", ModuleConstants.TURNING_GAINS.p)
  private val turnI = LoggedTunableNumber("$key/Turn/2.I", ModuleConstants.TURNING_GAINS.i)
  private val turnD = LoggedTunableNumber("$key/Turn/3.D", ModuleConstants.TURNING_GAINS.d)
  private val turnFF = LoggedTunableNumber("$key/Turn/4.FF", ModuleConstants.TURNING_GAINS.ff)

  init {
    module.setDesiredState(desiredState)

    LoggedTunableNumber.ifChanged(hashCode(), { values ->
      module.setDriveP(values[0])
      module.setDriveI(values[1])
      module.setDriveD(values[2])
      module.setDriveFF(values[3])
    }, driveP, driveI, driveD, driveFF)

    LoggedTunableNumber.ifChanged(hashCode(), { values ->
      module.setTurnP(values[0])
      module.setTurnI(values[1])
      module.setTurnD(values[2])
      module.setTurnFF(values[3])
    }, turnP, turnI, turnD, turnFF)
  }

  fun periodic() {
    module.updateInputs(inputs)
    Logger.processInputs(key, inputs)
  }

  fun setDesiredState(desiredState: SwerveModuleState) {
    this.desiredState = desiredState
    module.setDesiredState(desiredState)
  }

  fun getDesiredState() = desiredState

  fun getState() = module.getState()

  fun getPosition() = module.getPosition()

  fun resetDriveEncoder() = module.resetDriveEncoder()
}