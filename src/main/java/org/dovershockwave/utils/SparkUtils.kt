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
      runRelWithTimeout(RelSparkAction("Set Blocking") { spark, _, _ -> spark.setCANTimeout(BLOCKING_TIMEOUT) })
      actions.forEach { runRelWithTimeout(it) }
      runRelWithTimeout(RelSparkAction("Set Async") { spark, _, _ -> spark.setCANTimeout(ASYNC_TIMEOUT) })
    }

    fun CANSparkMax.configureRel(actions: LinkedHashSet<RelSparkAction>) {
      val configuration = LinkedHashSet<RelSparkAction>().apply {
        add(RelSparkAction("Restore Factory Default") { spark, _, _ -> spark.restoreFactoryDefaults() })
        addAll(actions)
        add(RelSparkAction("Burn Flash") { spark, _, _ -> spark.burnFlash() })
      }

      runBlockingRel(configuration)
    }

    private fun CANSparkMax.runRelWithTimeout(action: RelSparkAction, timeout: Int = TIMEOUT) {
      var count = 0;
      while (action.accept(this, encoder, pidController) != REVLibError.kOk && count < timeout) {
        // TODO: log the output w the name
        count++
      }

      // TODO: add something here i dont even know right now for the robot will not work.
    }

    fun CANSparkMax.runBlockingAbs(actions: LinkedHashSet<AbsSparkAction>) {
      runAbsWithTimeout(AbsSparkAction("Set Blocking") { spark, _, _ -> spark.setCANTimeout(BLOCKING_TIMEOUT) })
      actions.forEach { runAbsWithTimeout(it) }
      runAbsWithTimeout(AbsSparkAction("Set Async") { spark, _, _ -> spark.setCANTimeout(ASYNC_TIMEOUT) })
    }

    // FIXME: this should be blocking
    fun CANSparkMax.configureAbs(actions: LinkedHashSet<AbsSparkAction>) {
      val configuration = LinkedHashSet<AbsSparkAction>().apply {
        add(AbsSparkAction("Restore Factory Default") { spark, _, _ -> spark.restoreFactoryDefaults() })
        addAll(actions)
        add(AbsSparkAction("Burn Flash") { spark, _, _ -> spark.burnFlash() })
      }

      runBlockingAbs(configuration)
    }

    private fun CANSparkMax.runAbsWithTimeout(action: AbsSparkAction, timeout: Int = TIMEOUT) {
      var count = 0;
      while (action.accept(this, absoluteEncoder, pidController) != REVLibError.kOk && count < timeout) {
        // TODO: log the output w the name
        count++
      }

      // TODO: add something here i dont even know right now for the robot will not work.
    }
  }
}
