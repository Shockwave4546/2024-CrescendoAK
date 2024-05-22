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

package org.dovershockwave.subsystem.shooter

import org.dovershockwave.utils.PIDGains
import org.dovershockwave.utils.PositionConversionFactor

class ShooterConstants {
  companion object {
    const val BOTTOM_CAN_ID = 31
    const val TOP_CAN_ID = 30
    val REV_CONVERSION_FACTOR = PositionConversionFactor(1.0)
    const val RPS_CONVERSION_FACTOR = 1.0 / 60.0
    val BOT_GAINS = PIDGains(0.03, 0.0, 0.0, 0.011)
    val TOP_GAINS = PIDGains(0.03, 0.0, 0.0, 0.011)
    const val BOT_INVERTED = true
    const val TOP_INVERTED = true
    const val MIN_OUTPUT = -1.0
    const val MAX_OUTPUT = 1.0
    const val RPS_TOLERANCE = 2.5
    const val MIN_RPS = 0.0
    const val MAX_RPS = 90.0
  }
}