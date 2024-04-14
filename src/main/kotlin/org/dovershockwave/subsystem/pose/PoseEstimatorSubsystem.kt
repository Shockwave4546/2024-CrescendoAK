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

package org.dovershockwave.subsystem.pose

import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.dovershockwave.shuffleboard.ShuffleboardBoolean
import org.dovershockwave.subsystem.swerve.SwerveConstants
import org.dovershockwave.subsystem.swerve.SwerveSubsystem
import org.littletonrobotics.junction.Logger
import org.photonvision.PhotonPoseEstimator
import java.util.*
import kotlin.math.pow
import kotlin.math.sqrt

class PoseEstimatorSubsystem(private val poseEstimator: PoseEstimatorIO, private val swerve: SwerveSubsystem) : SubsystemBase() {
  private val inputs = PoseEstimatorIO.PoseEstimatorIOInputs()
  private val photonPoseEstimator: PhotonPoseEstimator = PhotonPoseEstimator(
    AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    poseEstimator.getRawFrontCamera(),
    PoseEstimatorConstants.FRONT_CAMERA_TO_ROBOT)
  private val swervePoseEstimator: SwerveDrivePoseEstimator
  private val useVisionMeasurement = ShuffleboardBoolean(Shuffleboard.getTab("PoseEstimator"), "Use Vision Measurement", false)

  init {
    photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)

    swervePoseEstimator = SwerveDrivePoseEstimator(
      SwerveConstants.DRIVE_KINEMATICS,
      Rotation2d(),
      arrayOf(
        SwerveModulePosition(0.0, Rotation2d()),
        SwerveModulePosition(0.0, Rotation2d()),
        SwerveModulePosition(0.0, Rotation2d()),
        SwerveModulePosition(0.0, Rotation2d()),
      ),
      Pose2d(),
      STATE_STD_DEVS,
      VISION_MEASUREMENT_STD_DEVS
    )
  }

  override fun periodic() {
    poseEstimator.updateInputs(inputs)

    swervePoseEstimator.update(swerve.getHeadingRotation2d(), swerve.getEstimatedPositions())
    if (useVisionMeasurement.get()) {
      photonPoseEstimator.update().ifPresent { estimatedPose ->
        swervePoseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), poseEstimator.getPipelineResults().timestampSeconds)
      }
    }

    val key = "PoseEstimator"
    Logger.processInputs(key, inputs)
    Logger.recordOutput("$key/EstimatedPose", getPose2d())
  }

  fun getDistanceToTag(id: Int): Optional<Double> {
    var distance = Optional.empty<Double>()
    for (target in poseEstimator.getPipelineResults().targets) {
      if (target.fiducialId != id) continue
      val camToTarget = target.bestCameraToTarget
      val robotToTarget = PoseEstimatorConstants.ROBOT_TO_FRONT_CAMERA.plus(camToTarget)

      val calcDistance = sqrt(robotToTarget.x.pow(2.0) + robotToTarget.y.pow(2.0))
      distance = Optional.of(calcDistance)
      break
    }

    return distance
  }

  fun getAngleToTag(id: Int): Optional<Rotation2d> {
    var angle = Optional.empty<Rotation2d>()
    for (target in poseEstimator.getPipelineResults().targets) {
      if (target.fiducialId != id) continue
      angle = Optional.of(Rotation2d.fromDegrees(target.yaw))
      break
    }

    return angle
  }

  fun isValidMeasurement(pose: Pose3d) = when {
    pose.x > PoseEstimatorConstants.FIELD_LENGTH -> {
      DriverStation.reportError("According to ${PoseEstimatorConstants.FRONT_CAMERA_NAME}, Robot is off the field in +x direction.", false)
      false
    }
    pose.x < 0 -> {
      DriverStation.reportError("According to ${PoseEstimatorConstants.FRONT_CAMERA_NAME}, Robot is off the field in -x direction.", false)
      false
    }
    pose.y > PoseEstimatorConstants.FIELD_WIDTH -> {
      DriverStation.reportError("According to ${PoseEstimatorConstants.FRONT_CAMERA_NAME}, Robot is off the field in +y direction.", false)
      false
    }
    pose.y < 0 -> {
      DriverStation.reportError("According to ${PoseEstimatorConstants.FRONT_CAMERA_NAME}, Robot is off the field in -y direction.", false)
      false
    }
    pose.z < -0.15 -> {
      DriverStation.reportError("According to ${PoseEstimatorConstants.FRONT_CAMERA_NAME}, Robot is inside the floor.", false)
      false
    }
    pose.z > 0.15 -> {
      DriverStation.reportError("According to ${PoseEstimatorConstants.FRONT_CAMERA_NAME}, Robot is floating above the floor.", false)
      false
    }
    else -> true
  }

  fun getPose2d() = swervePoseEstimator.estimatedPosition

  fun resetPose(pose: Pose2d) = swervePoseEstimator.resetPosition(swerve.getHeadingRotation2d(), swerve.getEstimatedPositions(), pose)

  fun resetFieldCentricDriving() = resetPose(Pose2d())

  companion object {
    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
     * Source: [...](https://github.com/STMARobotics/frc-7028-2023/blob/5916bb426b97f10e17d9dfd5ec6c3b6fda49a7ce/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java)
     */
    private val STATE_STD_DEVS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5.0))

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     * Source: [...](https://github.com/STMARobotics/frc-7028-2023/blob/5916bb426b97f10e17d9dfd5ec6c3b6fda49a7ce/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java)
     */
    private val VISION_MEASUREMENT_STD_DEVS = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10.0))
  }
}