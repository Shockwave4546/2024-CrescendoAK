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

import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import org.littletonrobotics.junction.Logger

/**
 * A class representing a boolean value on the Shuffleboard dashboard.
 */
class ShuffleboardBoolean(tab: ShuffleboardTab, private val name: String, private val def: Boolean = DEFAULT_VALUE) : ShuffleboardValue() {
  private val widget = tab.add(name, def)

  /**
   * Constructs a new ShuffleboardBoolean object with the given parameters.
   *
   * @param tab  the ShuffleboardTab to add the widget to
   * @param name the name of the widget
   * @param def  the default value for the widget
   */
  init {
    widget.withWidget(BuiltInWidgets.kToggleButton)
  }

  /**
   * Sets the size of the widget.
   *
   * @param length the length of the widget
   * @param height the height of the widget
   * @return the modified ShuffleboardBoolean object
   */
  fun withSize(length: Int, height: Int): ShuffleboardBoolean {
    widget.withSize(length, height)
    return this
  }

  /**
   * Sets the position of the widget.
   *
   * @param x the x coordinate of the widget's position
   * @param y the y coordinate of the widget's position
   * @return the modified ShuffleboardBoolean object
   */
  fun withPosition(x: Int, y: Int): ShuffleboardBoolean {
    widget.withPosition(x, y)
    return this
  }

  /**
   * Retrieves the current value of the ShuffleboardBoolean object.
   *
   * @return the current value of the ShuffleboardBoolean object
   */
  fun get() = widget.entry.getBoolean(def)

  /**
   * Sets the value of the ShuffleboardBoolean object.
   *
   * @param value the new value to set
   */
  fun set(value: Boolean) {
    Logger.recordOutput("DashboardInputs/$name", value)
    widget.entry.setBoolean(value)
  }

  /**
   * Returns the Raw GenericEntry object associated with this ShuffleboardBoolean.
   *
   * @return the Raw GenericEntry object
   */
  override fun getRaw(): GenericEntry {
    return widget.entry
  }

  companion object {
    private const val DEFAULT_VALUE = false
  }
}