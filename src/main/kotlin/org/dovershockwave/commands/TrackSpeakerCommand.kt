package org.dovershockwave.commands

import edu.wpi.first.wpilibj2.command.Command
import org.dovershockwave.subsystem.swerve.SwerveSubsystem
import org.dovershockwave.subsystem.vision.VisionSubsystem

class TrackSpeakerCommand(private val swerve: SwerveSubsystem, private val vision: VisionSubsystem) : Command() {
  init {
    addRequirements(swerve, vision)
  }

  override fun initialize() = swerve.stop()

  override fun execute() {
    val rot = vision.getToSpeakerFromVision().rotation2d
    if (rot.isEmpty) return
    val rotSpeed = -swerve.calculateSpeedForDesiredHeading(rot.get().radians)
    swerve.drive(0.0, 0.0, rotSpeed, fieldRelative = true, useDefaultSpeeds = false)
  }

  override fun isFinished() = vision.isHeadingAlignedWithSpeaker().isPresent && vision.isHeadingAlignedWithSpeaker().get()

  override fun end(interrupted: Boolean) = swerve.stop()
}