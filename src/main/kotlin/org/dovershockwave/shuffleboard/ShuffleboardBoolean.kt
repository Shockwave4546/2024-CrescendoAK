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

/**
 * A class representing a boolean value on the Shuffleboard dashboard.
 */
class ShuffleboardBoolean(
  tab: ShuffleboardTab,
  name: String,
  private val def: Boolean = org.dovershockwave.shuffleboard.ShuffleboardBoolean.Companion.DEFAULT_VALUE
) : org.dovershockwave.shuffleboard.ShuffleboardValue(name) {
  private val widget = tab.add(name, def)

  /**
   * Constructs a new ShuffleboardBoolean object with the given parameters.
   *
   * @param tab  the ShuffleboardTab to add the widget to
   * @param name the name of the widget
   * @param def  the default value for the widget
   */
  /**
   * Constructs a new ShuffleboardBoolean object with the given parameters. The default value for the widget is set to false.
   *
   * @param tab  the ShuffleboardTab to add the widget to
   * @param name the name of the widget
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
  fun withSize(length: Int, height: Int): org.dovershockwave.shuffleboard.ShuffleboardBoolean {
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
  fun withPosition(x: Int, y: Int): org.dovershockwave.shuffleboard.ShuffleboardBoolean {
    widget.withPosition(x, y)
    return this
  }

  /**
   * Retrieves the current value of the ShuffleboardBoolean object.
   *
   * @return the current value of the ShuffleboardBoolean object
   */
  fun getBoolean() = widget.entry.getBoolean(def)

  /**
   * Sets the value of the ShuffleboardBoolean object.
   *
   * @param value the new value to set
   */
  fun setBoolean(value: Boolean) {
    super.set(if (value) 1.0 else -1.0)
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