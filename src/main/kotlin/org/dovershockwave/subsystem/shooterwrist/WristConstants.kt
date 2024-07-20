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

package org.dovershockwave.subsystem.shooterwrist

import org.dovershockwave.utils.PIDGains
import org.dovershockwave.utils.PositionConversionFactor

class WristConstants {
  companion object {
    const val MOTOR_CAN_ID = 36
    val ANGLE_CONVERSION_FACTOR = PositionConversionFactor(PositionConversionFactor.ConversionType.DEGREES)

    // 7/20: val GAINS = PIDGains(0.02, 0.0, 0.005)
    val GAINS = PIDGains(0.043, 0.0, 0.007, 0.0005)

    const val ENCODER_INVERTED = false
    const val MIN_OUTPUT = -1.0
    const val MAX_OUTPUT = 1.0
    const val ANGLE_TOLERANCE = 0.5 // degrees

    // 7/20 - 113.5 to 120
    const val ANGLE_OFFSET = 120.0 // degrees

    const val MIN_ANGLE = 2.5 // degrees
    const val MAX_ANGLE = 80.0 // degrees
  }
}