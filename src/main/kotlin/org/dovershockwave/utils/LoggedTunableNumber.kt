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

import org.dovershockwave.GlobalConstants
import org.dovershockwave.RobotContainer
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber

class LoggedTunableNumber(dashboardKey: String, private val defaultValue: Double = 0.0) {
  private val key = "$TABLE_KEY/$dashboardKey"
  private var dashboardNumber: LoggedDashboardNumber? = null
  private val lastHasChangedValues = hashMapOf<Int, Double>()

  init {
    if (GlobalConstants.TUNING_MODE && !RobotContainer.isCompMatch()) {
      dashboardNumber = LoggedDashboardNumber(key, defaultValue)
    }
  }

  fun get() = dashboardNumber?.get() ?: defaultValue

  fun hasChanged(id: Int): Boolean {
    val currentValue = get()
    val lastValue = lastHasChangedValues[id]
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues[id] = currentValue
      return true
    }

    return false
  }

  companion object {
    private const val TABLE_KEY = "Tuning"

    fun ifChanged(id: Int, action: (DoubleArray) -> Unit, vararg tunableNumbers: LoggedTunableNumber) = tunableNumbers.filter { it.hasChanged(id) }.forEach { _ ->
      action(tunableNumbers.map { it.get() }.toDoubleArray())
    }

    fun ifChanged(id: Int, action: Runnable, vararg tunableNumbers: LoggedTunableNumber) = ifChanged(id, { _: DoubleArray -> action.run() }, *tunableNumbers)
  }
}