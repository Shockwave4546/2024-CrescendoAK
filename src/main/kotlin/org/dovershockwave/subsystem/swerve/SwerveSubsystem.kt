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

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.dovershockwave.Tabs
import org.dovershockwave.shuffleboard.ShuffleboardBoolean
import org.dovershockwave.shuffleboard.ShuffleboardSpeed
import org.dovershockwave.subsystem.swerve.gyro.GyroIO
import org.dovershockwave.subsystem.swerve.module.ModuleIO
import org.littletonrobotics.junction.Logger

class SwerveSubsystem(private val frontLeft: ModuleIO, private val frontRight: ModuleIO, private val backLeft: ModuleIO, private val backRight: ModuleIO, private val gyro: GyroIO) : SubsystemBase() {
  private val tab = Shuffleboard.getTab("Swerve")
  private val isX = ShuffleboardBoolean(Tabs.MATCH, "Is X?", false)
    .withSize(3, 3).withPosition(15, 0)

  private val driveSpeedMultiplier =
    ShuffleboardSpeed(tab, "Drive Speed Multiplier", SwerveConstants.DEFAULT_DRIVE_SPEED_MULTIPLIER)
      .withSize(5, 2).withPosition(0, 8)
  private val rotSpeedMultiplier =
    ShuffleboardSpeed(tab, "Rot Speed Multiplier", SwerveConstants.DEFAULT_ROT_SPEED_MULTIPLIER)
      .withSize(5, 2).withPosition(5, 8)
  private val isFieldRelative = ShuffleboardBoolean(
    Tabs.MATCH,
    "Is Field Relative?",
    true
  ).withSize(3, 3).withPosition(18, 0)

  private val gyroInputs = GyroIO.GyroIOInputs()
  private val frontLeftInputs = ModuleIO.ModuleIOInputs()
  private val frontRightInputs = ModuleIO.ModuleIOInputs()
  private val backLeftInputs = ModuleIO.ModuleIOInputs()
  private val backRightInputs = ModuleIO.ModuleIOInputs()

  init {
    resetDriveEncoders()
  }

  override fun periodic() {
    gyro.updateInputs(gyroInputs)
    frontLeft.updateInputs(frontLeftInputs)
    frontRight.updateInputs(frontRightInputs)
    backLeft.updateInputs(backLeftInputs)
    backRight.updateInputs(backRightInputs)

    Logger.processInputs("Gyro", gyroInputs)
    Logger.processInputs("Front Left", frontLeftInputs)
    Logger.processInputs("Front Right", frontRightInputs)
    Logger.processInputs("Back Left", backLeftInputs)
    Logger.processInputs("Back Right", backRightInputs)

    Logger.recordOutput("Swerve/ModuleStates", *getEstimatedStates())
    // Logger.recordOutput("Swerve/DesiredStates") TODO
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotSpeed      Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   * field.
   */
  fun drive(xSpeed: Double, ySpeed: Double, rotSpeed: Double, fieldRelative: Boolean, useDefaultSpeeds: Boolean) {
    if (isX.getBoolean()) {
      setX()
      return
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    val xSpeedDelivered =
      xSpeed * SwerveConstants.MAX_SPEED_METERS_PER_SECOND * (if (useDefaultSpeeds) SwerveConstants.DEFAULT_DRIVE_SPEED_MULTIPLIER else driveSpeedMultiplier.getDouble())
    val ySpeedDelivered =
      ySpeed * SwerveConstants.MAX_SPEED_METERS_PER_SECOND * (if (useDefaultSpeeds) SwerveConstants.DEFAULT_DRIVE_SPEED_MULTIPLIER else driveSpeedMultiplier.getDouble())
    val rotDelivered =
      rotSpeed * SwerveConstants.MAX_ANGULAR_SPEED * (if (useDefaultSpeeds) SwerveConstants.DEFAULT_ROT_SPEED_MULTIPLIER else rotSpeedMultiplier.getDouble())

    val swerveModuleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
      if (fieldRelative) ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeedDelivered,
        ySpeedDelivered,
        rotDelivered,
        getHeadingRotation2d()
      )
      else ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
    )

    setModuleStates(*swerveModuleStates)
  }

  /**
   * Overridden drive function for PathPlanner autonomous. It's also important to note that autonomous drives
   * given robot relative ChassisSpeeds (not field relative).
   *
   * @param speeds Speed to drive.
   */
  fun driveAutonomous(speeds: ChassisSpeeds) {
    val swerveModuleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds)
    setModuleStates(*swerveModuleStates)
  }

  /**
   * Stops the robot.
   */
  fun stop() = drive(0.0, 0.0, 0.0, fieldRelative = false, useDefaultSpeeds = false)

  /**
   * It's important the SwerveModules are passed in with respect to the Kinematics construction.
   *
   * @return chassis speed relative to the robot.
   */
  fun getRelativeChassisSpeed() = SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
    frontLeft.getState(),
    frontRight.getState(),
    backLeft.getState(),
    backRight.getState()
  )

  /**
   * @return the SwerveModulePositions of the SwerveModules.
   */
  fun getEstimatedPositions() = arrayOf(
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backLeft.getPosition(),
    backRight.getPosition()
  )

  fun getEstimatedStates() = arrayOf(
    frontLeft.getState(),
    frontRight.getState(),
    backLeft.getState(),
    backRight.getState()
  )

  /**
   * Sets the swerve ModuleStates. (FL, FR, BL, BR)
   *
   * @param desiredStates The desired SwerveModule states.
   */
  fun setModuleStates(vararg desiredStates: SwerveModuleState) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED_METERS_PER_SECOND)
    frontLeft.setDesiredState(desiredStates[0])
    frontRight.setDesiredState(desiredStates[1])
    backLeft.setDesiredState(desiredStates[2])
    backRight.setDesiredState(desiredStates[3])
  }

  private fun setX() {
    setModuleStates(
      SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
      SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
      SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
      SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0))
    )
  }

  fun toggleX() {
    isX.setBoolean(!isX.getBoolean())
  }

  fun isFieldRelative(): Boolean {
    return isFieldRelative.getBoolean()
  }

  fun resetDriveEncoders() {
    frontLeft.resetDriveEncoder()
    frontRight.resetDriveEncoder()
    backLeft.resetDriveEncoder()
    backRight.resetDriveEncoder()
  }

  /**
   * Zeroes the gyro of the robot.
   */
  fun zeroGyro() {
    gyro.reset()
  }

  /**
   * Returns the heading of the robot.
   *
   * @return The robot's heading as a Rotation2d.
   */
  fun getHeadingRotation2d() = Rotation2d.fromDegrees(getRawAngleDegrees())

  /**
   * The default SwerveMax template has an issue with inverting the Gyro, so the workaround is
   * manually negating the AHRS#getAngle. This function shouldn't get called by the user.
   *
   * @return The properly negated angle in degrees.
   */
  private fun getRawAngleDegrees() = (if (SwerveConstants.GYRO_REVERSED) -1.0 else 1.0) * gyro.getAngle()
}