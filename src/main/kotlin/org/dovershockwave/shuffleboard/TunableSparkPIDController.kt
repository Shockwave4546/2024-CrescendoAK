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

package org.dovershockwave.shuffleboard

import com.revrobotics.SparkPIDController
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier

// TODO: Probably log this.

/**
 * Wraps [SparkPIDController] because it isn't natively supported by Shuffleboard.
 */
class TunableSparkPIDController(private val child: SparkPIDController, private val setpointGetter: DoubleSupplier? = null, private val setpointSetter: DoubleConsumer? = null) : Sendable {
  /**
   * Initializes the Sendable interface for the TunableSparkMaxPIDController.
   *
   * @param builder the Sendable builder used to configure the SmartDashboard properties
   */
  override fun initSendable(builder: SendableBuilder) {
    builder.setSmartDashboardType("PIDController")
    builder.addDoubleProperty("p", { child.p }, { gain -> child.setP(gain) })
    builder.addDoubleProperty("i", { child.i }, { gain -> child.setI(gain) })
    builder.addDoubleProperty("d", { child.d }, { gain -> child.setD(gain) })
    builder.addDoubleProperty("f", { child.ff }, { gain -> child.setFF(gain) })
    builder.addDoubleProperty("setpoint", setpointGetter, setpointSetter)
    // SparkPIDController doesn't have an option to disable it, so the controller is always enabled.
    builder.addBooleanProperty("enabled", { true }, null)
  }
}