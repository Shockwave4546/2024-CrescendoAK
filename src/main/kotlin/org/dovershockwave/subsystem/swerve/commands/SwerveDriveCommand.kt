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

package org.dovershockwave.subsystem.swerve.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.dovershockwave.GlobalConstants
import org.dovershockwave.subsystem.swerve.SwerveSubsystem
import org.dovershockwave.subsystem.vision.VisionSubsystem

class SwerveDriveCommand(private val controller: CommandXboxController, private val swerve: SwerveSubsystem, private val vision: VisionSubsystem) : Command() {
  init {
    addRequirements(swerve)
  }

  override fun execute() {
    var rotSpeed = -MathUtil.applyDeadband(controller.rightX, GlobalConstants.DRIVE_DEADBAND)

    if (swerve.isAutoAlign()) {
      val rot = vision.getToSpeakerFromVision().rotation2d
      if (rot.isPresent) {
        rotSpeed = swerve.calculateSpeedForDesiredHeading(rot.get().radians)
      }
    }

    swerve.drive(
      -MathUtil.applyDeadband(controller.leftY, GlobalConstants.DRIVE_DEADBAND),
      -MathUtil.applyDeadband(controller.leftX, GlobalConstants.DRIVE_DEADBAND),
      rotSpeed,
      swerve.isFieldRelative(),
      useConstDriveSpeed = false,
      useConstRotSpeed = swerve.isAutoAlign()
    )
  }
}