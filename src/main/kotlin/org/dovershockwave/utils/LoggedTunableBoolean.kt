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
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean

class LoggedTunableBoolean(dashboardKey: String, private val defaultValue: Boolean = false) {
  private val key = "$TABLE_KEY/$dashboardKey"
  private var dashboardBoolean: LoggedDashboardBoolean? = null
  private val lastHasChangedValues = hashMapOf<Int, Boolean>()

  init {
    if (GlobalConstants.TUNING_MODE) {
      dashboardBoolean = LoggedDashboardBoolean(key, defaultValue)
    }
  }

  fun get() = dashboardBoolean?.get() ?: defaultValue

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

    fun ifChanged(id: Int, action: (BooleanArray) -> Unit, vararg tunableBooleans: LoggedTunableBoolean) {
      tunableBooleans.filter { it.hasChanged(id) }.forEach { _ ->
        action(tunableBooleans.map { it.get() }.toBooleanArray())
      }
    }

    fun ifChanged(id: Int, action: Runnable, vararg tunableBooleans: LoggedTunableBoolean) = ifChanged(id, { _: BooleanArray -> action.run() }, *tunableBooleans)
  }
}