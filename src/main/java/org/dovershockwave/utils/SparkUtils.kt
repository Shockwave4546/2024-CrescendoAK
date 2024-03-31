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
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import com.revrobotics.SparkPIDController

class SparkUtils {
  companion object {
    fun runBlockingRel(spark: CANSparkMax, action: (CANSparkMax, RelativeEncoder, SparkPIDController) -> Unit) {
      spark.setCANTimeout(250)
      action(spark, spark.encoder, spark.pidController)
      spark.setCANTimeout(0)
    }

    fun configureRel(spark: CANSparkMax, action: (CANSparkMax, RelativeEncoder, SparkPIDController) -> Unit) {
      runBlockingRel(spark) { newSpark, encoder, pid ->
        newSpark.restoreFactoryDefaults()
        action(newSpark, encoder, pid)
        newSpark.burnFlash()
      }
    }

    fun runBlockingAbs(spark: CANSparkMax, action: (CANSparkMax, AbsoluteEncoder, SparkPIDController) -> Unit) {
      spark.setCANTimeout(250)
      action(spark, spark.absoluteEncoder, spark.pidController)
      spark.setCANTimeout(0)
    }

    fun configureAbs(spark: CANSparkMax, action: (CANSparkMax, AbsoluteEncoder, SparkPIDController) -> Unit) {
      runBlockingAbs(spark) { newSpark, encoder, pid ->
        newSpark.restoreFactoryDefaults()
        action(newSpark, encoder, pid)
        newSpark.burnFlash()
      }
    }
  }
}

fun CANSparkMax.runBlockingRel(action: (CANSparkMax, RelativeEncoder, SparkPIDController) -> Unit) {
  SparkUtils.runBlockingRel(this, action)
}

fun CANSparkMax.configureRel(action: (CANSparkMax, RelativeEncoder, SparkPIDController) -> Unit) {
  SparkUtils.configureRel(this, action)
}

fun CANSparkMax.runBlockingAbs(action: (CANSparkMax, AbsoluteEncoder, SparkPIDController) -> Unit) {
  SparkUtils.runBlockingAbs(this, action)
}

fun CANSparkMax.configureAbs(action: (CANSparkMax, AbsoluteEncoder, SparkPIDController) -> Unit) {
  SparkUtils.configureAbs(this, action)
}