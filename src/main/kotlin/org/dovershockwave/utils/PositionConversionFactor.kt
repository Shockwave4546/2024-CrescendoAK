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

package org.dovershockwave.utils

import com.revrobotics.AbsoluteEncoder
import com.revrobotics.RelativeEncoder
import edu.wpi.first.wpilibj.Encoder

/**
 * Represents a universal conversion factor for Encoders throughout a SparkMaxRelativeEncoder vs a Quadrature Encoder.
 */
class PositionConversionFactor(private val oneRev: Double) {
  /**
   * Represents some default supported types for conversion through the Encoders.
   */
  object ConversionType {
    const val DEGREES = 360.0
    const val RADIANS = 2 * Math.PI
  }

  /**
   * @param encoder the SparkMaxRelativeEncoder to perform the transformation.
   */
  fun apply(encoder: RelativeEncoder) = encoder.setPositionConversionFactor(toSparkMaxRelativeEncoder())!!

  /**
   * @param encoder the SparkMaxAbsoluteEncoder to perform the transformation.
   */
  fun apply(encoder: AbsoluteEncoder) = encoder.setPositionConversionFactor(toSparkMaxRelativeEncoder())!!

  /**
   * @param encoder the Encoder to perform the transformation.
   */
  fun apply(encoder: Encoder, inverted: Boolean) {
    encoder.distancePerPulse = toQuadEncoder()
    encoder.setReverseDirection(inverted)
  }

  /**
   * Note: a SparkMaxRelativeEncoder has the native units of revolutions, thus the extra division isn't needed.
   *
   * @return the conversion factor for a SparkMaxRelativeEncoder.
   */
  private fun toSparkMaxRelativeEncoder() = oneRev

  /**
   * @return the conversion factor for a Quadrature Encoder.
   */
  private fun toQuadEncoder() = oneRev / QUAD_PULSES_PER_REV

  companion object {
    private const val QUAD_PULSES_PER_REV = 2048.0
  }
}