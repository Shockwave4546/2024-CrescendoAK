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

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.dovershockwave.Tab
import org.dovershockwave.shuffleboard.ShuffleboardBoolean
import org.dovershockwave.shuffleboard.ShuffleboardSpeed
import org.dovershockwave.subsystem.swerve.gyro.GyroIO
import org.dovershockwave.subsystem.swerve.module.Module
import org.dovershockwave.subsystem.vision.VisionConstants
import org.dovershockwave.utils.LoggedTunableNumber
import org.littletonrobotics.junction.Logger

class SwerveSubsystem(private val frontLeft: Module, private val frontRight: Module, private val backLeft: Module, private val backRight: Module, private val gyro: GyroIO) : SubsystemBase() {
  private val isX = ShuffleboardBoolean(Tab.MATCH, "Is X?", false).withSize(3, 3)

  private val driveSpeedMultiplier = ShuffleboardSpeed(Tab.MATCH, "Drive Speed Multiplier", SwerveConstants.DEFAULT_DRIVE_SPEED_MULTIPLIER).withSize(5, 2)
  private val rotSpeedMultiplier = ShuffleboardSpeed(Tab.MATCH, "Rot Speed Multiplier", SwerveConstants.DEFAULT_ROT_SPEED_MULTIPLIER).withSize(5, 2)
  private val isFieldRelative = ShuffleboardBoolean(Tab.MATCH, "Is Field Relative?", true).withSize(3, 3)

  private val autoAlign = ShuffleboardBoolean(Tab.MATCH, "Auto Align", false).withSize(3, 3)

  private val headingController = ProfiledPIDController(
    SwerveConstants.HEADING_GAINS.p,
    SwerveConstants.HEADING_GAINS.i,
    SwerveConstants.HEADING_GAINS.d,
    TrapezoidProfile.Constraints(
      Units.degreesToRadians(540.0),
      Units.degreesToRadians(720.0)
    )
  )

  private val key = "Swerve"
  private val p = LoggedTunableNumber("$key/Heading/1.P", SwerveConstants.HEADING_GAINS.p)
  private val i = LoggedTunableNumber("$key/Heading/2.I", SwerveConstants.HEADING_GAINS.i)
  private val d = LoggedTunableNumber("$key/Heading/3.D", SwerveConstants.HEADING_GAINS.d)

  private val gyroInputs = GyroIO.GyroIOInputs()

  init {
    resetDriveEncoders()

    headingController.setTolerance(VisionConstants.HEADING_TOLERANCE)
  }

  override fun periodic() {
    frontLeft.periodic()
    frontRight.periodic()
    backLeft.periodic()
    backRight.periodic()

    gyro.updateInputs(gyroInputs)
    Logger.processInputs("Gyro", gyroInputs)

    LoggedTunableNumber.ifChanged(hashCode(), { values ->
      headingController.p = values[0]
      headingController.i = values[1]
      headingController.d = values[2]
    }, p, i, d)

    Logger.recordOutput("$key/1.ModuleStates", *getEstimatedStates())
    Logger.recordOutput("$key/2.DesiredStates", *getDesiredStates())
    Logger.recordOutput("$key/3.XVelocity", getRelativeChassisSpeed().vxMetersPerSecond)
    Logger.recordOutput("$key/4.YVelocity", getRelativeChassisSpeed().vyMetersPerSecond)
    Logger.recordOutput("$key/5.Rotation2d", getHeadingRotation2d())
    Logger.recordOutput("$key/6.Rotation2d degrees", getHeadingRotation2d().degrees)
    Logger.recordOutput("$key/7.Rotation2d rad", getHeadingRotation2d().radians)
  }

  fun isAutoAlign() = autoAlign.get()

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
    if (isX.get()) {
      setX()
      return
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    val xSpeedDelivered = xSpeed * SwerveConstants.MAX_SPEED_METERS_PER_SECOND * (if (useDefaultSpeeds) SwerveConstants.DEFAULT_DRIVE_SPEED_MULTIPLIER else driveSpeedMultiplier.get())
    val ySpeedDelivered = ySpeed * SwerveConstants.MAX_SPEED_METERS_PER_SECOND * (if (useDefaultSpeeds) SwerveConstants.DEFAULT_DRIVE_SPEED_MULTIPLIER else driveSpeedMultiplier.get())
    val rotDelivered = rotSpeed * SwerveConstants.MAX_ANGULAR_SPEED * (if (useDefaultSpeeds) SwerveConstants.DEFAULT_ROT_SPEED_MULTIPLIER else rotSpeedMultiplier.get())

    val swerveModuleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
      if (fieldRelative) ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeedDelivered,
        ySpeedDelivered,
        rotDelivered,
        getHeadingRotation2d()
      )
      else ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
    )

    setDesiredModuleStates(*swerveModuleStates)
  }


  /**
   * Overridden drive function for PathPlanner autonomous. It's also important to note that autonomous drives
   * given robot relative ChassisSpeeds (not field relative).
   *
   * @param speeds Speed to drive.
   */
  fun driveAutonomous(speeds: ChassisSpeeds) {
    val swerveModuleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds)
    setDesiredModuleStates(*swerveModuleStates)
  }

  fun calculateSpeedForDesiredHeading(desiredHeading: Double): Double {
    headingController.setGoal(0.0)
    return headingController.calculate(desiredHeading)
  }

  fun setMaxSpeed(maxDrive: Double, maxRot: Double) {
    driveSpeedMultiplier.set(maxDrive)
    rotSpeedMultiplier.set(maxRot)
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

  fun getDesiredStates() = arrayOf(
    frontLeft.getDesiredState(),
    frontRight.getDesiredState(),
    backLeft.getDesiredState(),
    backRight.getDesiredState()
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
  fun setDesiredModuleStates(vararg desiredStates: SwerveModuleState) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED_METERS_PER_SECOND)
    frontLeft.setDesiredState(desiredStates[0])
    frontRight.setDesiredState(desiredStates[1])
    backLeft.setDesiredState(desiredStates[2])
    backRight.setDesiredState(desiredStates[3])
  }

  private fun setX() {
    setDesiredModuleStates(
      SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
      SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
      SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
      SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0))
    )
  }

  fun toggleX() {
    isX.set(!isX.get())
  }

  fun isFieldRelative() = isFieldRelative.get()

  fun resetDriveEncoders() {
    frontLeft.resetDriveEncoder()
    frontRight.resetDriveEncoder()
    backLeft.resetDriveEncoder()
    backRight.resetDriveEncoder()
  }

  /**
   * Zeroes the gyro of the robot.
   */
  fun zeroGyro() = gyro.reset()

  /**
   * Returns the heading of the robot.
   *
   * @return The robot's heading as a Rotation2d.
   */
  fun getHeadingRotation2d(): Rotation2d = Rotation2d.fromDegrees(getRawAngleDegrees())

  /**
   * The default SwerveMax template has an issue with inverting the Gyro, so the workaround is
   * manually negating the AHRS#getAngle. This function shouldn't get called by the user.
   *
   * @return The properly negated angle in degrees.
   */
  private fun getRawAngleDegrees() = (if (SwerveConstants.GYRO_REVERSED) -1.0 else 1.0) * gyro.getAngle()
}