package org.dovershockwave.subsystem.swerve

import org.dovershockwave.subsystem.swerve.module.Module
import org.dovershockwave.subsystem.swerve.module.ModuleIOSim

class SwerveIOSim : SwerveIO {
  override val frontLeft = Module(
    ModuleIOSim(SwerveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET),
    "FL"
  )

  override val frontRight = Module(
    ModuleIOSim(SwerveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET),
    "FR"
  )

  override val backLeft = Module(
    ModuleIOSim(SwerveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET),
    "BL"
  )

  override val backRight = Module(
    ModuleIOSim(SwerveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET),
    "BR"
  )
}