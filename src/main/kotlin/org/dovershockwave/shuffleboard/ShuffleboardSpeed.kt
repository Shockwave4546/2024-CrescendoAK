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

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab

/**
 * A Shuffleboard control for adjusting the speed value.
 */
class ShuffleboardSpeed(tab: ShuffleboardTab, name: String, def: Double = DEFAULT_VALUE) :
  org.dovershockwave.shuffleboard.ShuffleboardDouble(tab, name, def) {
  /**
   * Creates a ShuffleboardSpeed widget with the given name and default value,
   * and adds it to the specified Shuffleboard tab.
   *
   * @param tab the Shuffleboard tab to add the widget to
   * @param name the name of the ShuffleboardSpeed widget
   * @param def the default value of the ShuffleboardSpeed widget
   */
  /**
   * Creates a ShuffleboardSpeed widget with the given name and a default value,
   * and adds it to the specified Shuffleboard tab.
   *
   * @param tab the Shuffleboard tab to add the widget to
   * @param name the name of the ShuffleboardSpeed widget
   */
  init {
    withMinMax(-1.0, 1.0)
  }

  /**
   * Sets the size of the widget.
   *
   * @param length the length of the widget
   * @param height the height of the widget
   * @return the modified ShuffleboardDouble object
   */
  override fun withSize(length: Int, height: Int): ShuffleboardSpeed {
    super.withSize(length, height)
    return this
  }

  /**
   * Sets the position of the widget.
   *
   * @param x the x coordinate of the widget's position
   * @param y the y coordinate of the widget's position
   * @return the modified ShuffleboardDouble object
   */
  override fun withPosition(x: Int, y: Int): ShuffleboardSpeed {
    super.withPosition(x, y)
    return this
  }

  companion object {
    private const val DEFAULT_VALUE = 0.0
  }
}