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

package org.dovershockwave.subsystem.vision

import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units

class VisionConstants {
  companion object {
    /**
     * Physical location of the camera on the robot, relative to the center of the robot.
     */
    val FRONT_CAMERA_TO_ROBOT = Transform3d(Translation3d(0.08, 0.15, 0.0), Rotation3d(0.0, 0.0, 55.0))
    val ROBOT_TO_FRONT_CAMERA = FRONT_CAMERA_TO_ROBOT.inverse()!!

    const val MAXIMUM_AMBIGUITY = 0.5
    const val FRONT_CAMERA_NAME = "FrontCamera"

    const val FIELD_LENGTH = 16.54175
    const val FIELD_WIDTH = 8.0137

    const val RED_SPEAKER_ID = 4
    const val BLUE_SPEAKER_ID = 7

    val HEADING_TOLERANCE = Units.degreesToRadians(2.0)

    val SHOOTABLE_DISTANCE = 3.1
  }
}