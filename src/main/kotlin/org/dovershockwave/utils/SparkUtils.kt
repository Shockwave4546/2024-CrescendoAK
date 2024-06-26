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
import org.littletonrobotics.junction.Logger

class RelSparkAction(val name: String, private val action: (CANSparkMax, RelativeEncoder, SparkPIDController) -> REVLibError) : TriFunction<CANSparkMax, RelativeEncoder, SparkPIDController, REVLibError> {
  override fun accept(in1: CANSparkMax, in2: RelativeEncoder, in3: SparkPIDController) = action(in1, in2, in3)
}

class AbsSparkAction(val name: String, private val action: (CANSparkMax, AbsoluteEncoder, SparkPIDController) -> REVLibError) : TriFunction<CANSparkMax, AbsoluteEncoder, SparkPIDController, REVLibError> {
  override fun accept(in1: CANSparkMax, in2: AbsoluteEncoder, in3: SparkPIDController) = action(in1, in2, in3)
}

class SparkUtils {
  companion object {
    private const val BLOCKING_TIMEOUT = 250
    private const val ASYNC_TIMEOUT = 0
    private const val TIMEOUT = 5

    fun CANSparkMax.runBlockingRel(actions: LinkedHashSet<RelSparkAction>) {
      runRelWithTimeout(RelSparkAction("$deviceId Set Blocking") { spark, _, _ -> spark.setCANTimeout(BLOCKING_TIMEOUT) })
      actions.forEach { runRelWithTimeout(it) }
      runRelWithTimeout(RelSparkAction("$deviceId Set Async") { spark, _, _ -> spark.setCANTimeout(ASYNC_TIMEOUT) })
    }

    fun CANSparkMax.configureRel(actions: LinkedHashSet<RelSparkAction>) {
      val configuration = LinkedHashSet<RelSparkAction>().apply {
        add(RelSparkAction("$deviceId Restore Factory Default") { spark, _, _ -> spark.restoreFactoryDefaults() })
        addAll(actions)
        add(RelSparkAction("$deviceId Burn Flash") { spark, _, _ -> spark.burnFlash() })
      }

      runBlockingRel(configuration)
    }

    private fun CANSparkMax.runRelWithTimeout(action: RelSparkAction, timeout: Int = TIMEOUT) {
      var count = 1
      var output = action.accept(this, encoder, pidController)
      Logger.recordOutput("SparkMaxActions/${action.name} #0", output)
      while (output != REVLibError.kOk && count < timeout) {
        Logger.recordOutput("SparkMaxActions/${action.name} #$count", output)
        output = action.accept(this, encoder, pidController)
        count++
      }

      if (output == REVLibError.kOk) return
      throw RuntimeException("Failed to run action: ${action.name}")
    }

    fun CANSparkMax.runBlockingAbs(actions: LinkedHashSet<AbsSparkAction>) {
      runAbsWithTimeout(AbsSparkAction("$deviceId Set Blocking") { spark, _, _ -> spark.setCANTimeout(BLOCKING_TIMEOUT) })
      actions.forEach { runAbsWithTimeout(it) }
      runAbsWithTimeout(AbsSparkAction("$deviceId Set Async") { spark, _, _ -> spark.setCANTimeout(ASYNC_TIMEOUT) })
    }

    fun CANSparkMax.configureAbs(actions: LinkedHashSet<AbsSparkAction>) {
      val configuration = LinkedHashSet<AbsSparkAction>().apply {
        add(AbsSparkAction("$deviceId Restore Factory Default") { spark, _, _ -> spark.restoreFactoryDefaults() })
        addAll(actions)
        add(AbsSparkAction("$deviceId Burn Flash") { spark, _, _ -> spark.burnFlash() })
      }

      runBlockingAbs(configuration)
    }

    private fun CANSparkMax.runAbsWithTimeout(action: AbsSparkAction, timeout: Int = TIMEOUT) {
      var count = 1
      var output = action.accept(this, absoluteEncoder, pidController)
      Logger.recordOutput("SparkMaxActions/${action.name} #0", output)
      while (output != REVLibError.kOk && count < timeout) {
        Logger.recordOutput("SparkMaxActions/${action.name} #$count", output)
        output = action.accept(this, absoluteEncoder, pidController)
        count++
      }

      if (output == REVLibError.kOk) return
      throw RuntimeException("Failed to run action: ${action.name}")
    }
  }
}
