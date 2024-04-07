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

import com.revrobotics.*

class RelSparkAction(val name: String, private val action: (CANSparkMax, RelativeEncoder, SparkPIDController) -> REVLibError) : TriFunction<CANSparkMax, RelativeEncoder, SparkPIDController, REVLibError> {
  override fun accept(in1: CANSparkMax, in2: RelativeEncoder, in3: SparkPIDController) = action(in1, in2, in3)
}

class AbsSparkAction(val name: String, private val action: (CANSparkMax, AbsoluteEncoder, SparkPIDController) -> REVLibError) : TriFunction<CANSparkMax, AbsoluteEncoder, SparkPIDController, REVLibError> {
  override fun accept(in1: CANSparkMax, in2: AbsoluteEncoder, in3: SparkPIDController) = action(in1, in2, in3)
}

class SparkUtils {
  companion object {
    private const val BLOCKING_TIMEOUT = 250;
    private const val ASYNC_TIMEOUT = 0;
    private const val TIMEOUT = 5;

    fun CANSparkMax.runBlockingRel(actions: LinkedHashSet<RelSparkAction>) {
      performRelWithTimeout(RelSparkAction("Set Blocking") { newSpark, _, _ -> newSpark.setCANTimeout(BLOCKING_TIMEOUT) })
      actions.forEach { performRelWithTimeout(it) }
      performRelWithTimeout(RelSparkAction("Set Async") { newSpark, _, _ -> newSpark.setCANTimeout(ASYNC_TIMEOUT) })
    }

    fun CANSparkMax.configureRel(actions: LinkedHashSet<RelSparkAction>) {
      performRelWithTimeout(RelSparkAction("Restore Factory Default") { newSpark, _, _ -> newSpark.restoreFactoryDefaults() })
      actions.forEach { performRelWithTimeout(it) }
      performRelWithTimeout(RelSparkAction("Burn Flash") { newSpark, _, _ -> newSpark.burnFlash() })
    }

    private fun CANSparkMax.performRelWithTimeout(action: RelSparkAction, timeout: Int = TIMEOUT) {
      var count = 0;
      while (action.accept(this, encoder, pidController) != REVLibError.kOk && count < timeout) {
        // TODO: log the output w the name
        count++
      }

      // TODO: add something here i dont even know right now for the robot will not work.
    }

    fun CANSparkMax.runBlockingAbs(actions: LinkedHashSet<AbsSparkAction>) {
      performAbsWithTimeout(AbsSparkAction("Set Blocking") { newSpark, _, _ -> newSpark.setCANTimeout(BLOCKING_TIMEOUT) })
      actions.forEach { performAbsWithTimeout(it) }
      performAbsWithTimeout(AbsSparkAction("Set Async") { newSpark, _, _ -> newSpark.setCANTimeout(ASYNC_TIMEOUT) })
    }

    fun CANSparkMax.configureAbs(actions: LinkedHashSet<AbsSparkAction>) {
      performAbsWithTimeout(AbsSparkAction("Restore Factory Default") { newSpark, _, _ -> newSpark.restoreFactoryDefaults() })
      actions.forEach { performAbsWithTimeout(it) }
      performAbsWithTimeout(AbsSparkAction("Burn Flash") { newSpark, _, _ -> newSpark.burnFlash() })
    }

    private fun CANSparkMax.performAbsWithTimeout(action: AbsSparkAction, timeout: Int = TIMEOUT) {
      var count = 0;
      while (action.accept(this, absoluteEncoder, pidController) != REVLibError.kOk && count < timeout) {
        // TODO: log the output w the name
        count++
      }

      // TODO: add something here i dont even know right now for the robot will not work.
    }
  }
}
