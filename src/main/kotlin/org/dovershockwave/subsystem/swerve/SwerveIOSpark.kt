package org.dovershockwave.subsystem.swerve

import org.dovershockwave.subsystem.swerve.module.Module
import org.dovershockwave.subsystem.swerve.module.ModuleIOSpark

class SwerveIOSpark : SwerveIO {
  override val frontLeft = Module(
    ModuleIOSpark(
      SwerveConstants.FRONT_LEFT_DRIVING_CAN_ID,
      SwerveConstants.FRONT_LEFT_TURNING_CAN_ID,
      SwerveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET
    ),
    "FL"
  )

  override val frontRight = Module(
    ModuleIOSpark(
      SwerveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
      SwerveConstants.FRONT_RIGHT_TURNING_CAN_ID,
      SwerveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET
    ),
    "FR"
  )

  override val backLeft = Module(
    ModuleIOSpark(
      SwerveConstants.BACK_LEFT_DRIVING_CAN_ID,
      SwerveConstants.BACK_LEFT_TURNING_CAN_ID,
      SwerveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET
    ),
    "BL"
  )

  override val backRight = Module(
    ModuleIOSpark(
      SwerveConstants.BACK_RIGHT_DRIVING_CAN_ID,
      SwerveConstants.BACK_RIGHT_TURNING_CAN_ID,
      SwerveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET
    ),
    "BR"
  )
}