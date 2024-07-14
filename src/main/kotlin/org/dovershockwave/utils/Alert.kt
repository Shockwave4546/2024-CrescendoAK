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

import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

/** Class for managing persistent alerts to be sent over NetworkTables.  */
class Alert(group: String = "Alerts", text: String, type: AlertType) {
  private val type: AlertType
  private var active = false
  private var activeStartTime = 0.0
  private var text: String

  /**
   * Creates a new Alert. If this is the first to be instantiated in its group, the appropriate
   * entries will be added to NetworkTables.
   */
  init {
    if (!GROUPS.containsKey(group)) {
      GROUPS[group] = SendableAlerts()
      SmartDashboard.putData(group, GROUPS[group])
    }

    this.text = text
    this.type = type
    GROUPS[group]!!.alerts.add(this)
  }

  /**
   * Sets whether the alert should currently be displayed. When activated, the alert text will also
   * be sent to the console.
   */
  fun set(active: Boolean) {
    if (active && !this.active) {
      activeStartTime = Timer.getFPGATimestamp()
      when (type) {
        AlertType.ERROR -> DriverStation.reportError(text, false)
        AlertType.WARNING -> DriverStation.reportWarning(text, false)
        AlertType.INFO -> println(text)
      }
    }

    this.active = active
  }

  /** Updates current alert text.  */
  fun setText(text: String) {
    if (active && text != this.text) {
      when (type) {
        AlertType.ERROR -> DriverStation.reportError(text, false)
        AlertType.WARNING -> DriverStation.reportWarning(text, false)
        AlertType.INFO -> println(text)
      }
    }
    this.text = text
  }

  private class SendableAlerts : Sendable {
    val alerts: MutableList<Alert> = ArrayList()

    fun getStrings(type: AlertType) = alerts.filter { alert -> alert.type == type && alert.active }
      .sortedBy { alert -> alert.activeStartTime }
      .map { alert -> alert.text }
      .toTypedArray()

    override fun initSendable(builder: SendableBuilder) {
      builder.setSmartDashboardType("Alerts")
      builder.addStringArrayProperty("errors", { getStrings(AlertType.ERROR) }, null)
      builder.addStringArrayProperty("warnings", { getStrings(AlertType.WARNING) }, null)
      builder.addStringArrayProperty("infos", { getStrings(AlertType.INFO) }, null)
    }
  }

  /** Represents an alert's level of urgency.  */
  enum class AlertType {
    /**
     * High priority alert - displayed first on the dashboard with a red "X" symbol. Use this type
     * for problems which will seriously affect the robot's functionality and thus require immediate
     * attention.
     */
    ERROR,

    /**
     * Medium priority alert - displayed second on the dashboard with a yellow "!" symbol. Use this
     * type for problems which could affect the robot's functionality but do not necessarily require
     * immediate attention.
     */
    WARNING,

    /**
     * Low priority alert - displayed last on the dashboard with a green "i" symbol. Use this type
     * for problems which are unlikely to affect the robot's functionality, or any other alerts
     * which do not fall under "ERROR" or "WARNING".
     */
    INFO
  }

  companion object {
    private val GROUPS: MutableMap<String, SendableAlerts> = HashMap()
  }
}