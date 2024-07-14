package org.dovershockwave.subsystem.swerve

import org.dovershockwave.subsystem.swerve.module.Module

interface SwerveIO {
  val frontLeft: Module
  val frontRight: Module
  val backLeft: Module
  val backRight: Module

  fun getModules() = arrayOf(frontLeft, frontRight, backLeft, backRight)
}