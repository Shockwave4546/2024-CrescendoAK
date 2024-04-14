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

package org.dovershockwave.subsystem.led

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class LEDSubsystem : SubsystemBase() {
  private val led = AddressableLED(LEDConstants.PWM_ID)
  private val buffer = AddressableLEDBuffer(LEDConstants.BUFFER_LENGTH)
  private val state = LEDState.RAINBOW

  init {
    led.setLength(buffer.length)
    led.start()
  }

  override fun periodic() {
    state.action(this)
    Logger.recordOutput("LED/State", state.toString())
  }

  /**
   * Source: [...](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html)
   */
  private var rainbowFirstPixelHue = 0
  fun rainbow() {
    for (i in 0 until buffer.length) {
      val hue = (rainbowFirstPixelHue + (i * 180 / buffer.length)) % 180
      buffer.setHSV(i, hue, 255, 128)
    }

    rainbowFirstPixelHue += 3
    rainbowFirstPixelHue %= 180
  }

  fun setStaticColor(color: Color) {
    for (i in 0 until buffer.length) {
      buffer.setLED(i, color)
    }
  }

  fun stop() {
    setStaticColor(LEDConstants.OFF)
    led.stop()
  }
}