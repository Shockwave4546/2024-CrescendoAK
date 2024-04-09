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

package org.dovershockwave

import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.DriverStation
import org.littletonrobotics.junction.LogDataReceiver
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.LogTable.LogValue
import org.littletonrobotics.junction.LogTable.LoggableType

class NTSafePublisher : LogDataReceiver {
  private val rootTable: NetworkTable = NetworkTableInstance.getDefault().getTable("/AdvantageKit")
  private var lastTable = LogTable(0)
  private val timestampPublisher: IntegerPublisher =
    rootTable
      .getIntegerTopic(LogDataReceiver.timestampKey.substring(1))
      .publish(PubSubOption.sendAll(true))
  private val publishers: MutableMap<String, GenericPublisher?> = HashMap()

  override fun putTable(table: LogTable) {
    // only stream data if we are not connected to FMS to reduce bandwidth issues
    if (!DriverStation.isFMSAttached()) {
      // Send timestamp
      timestampPublisher[table.timestamp] = table.timestamp

      // Get old and new data
      val newMap = table.getAll(false)
      val oldMap = lastTable.getAll(false)

      // Encode new/changed fields
      for (field: Map.Entry<String, LogValue> in newMap) {
        // Check if field has changed
        val newValue = field.value
        if (newValue == oldMap[field.key]) {
          continue
        }

        // Create publisher if necessary
        val key = field.key.substring(1)
        var publisher = publishers[key]
        if (publisher == null) {
          publisher =
            rootTable
              .getTopic(key)
              .genericPublish(newValue.type.nT4Type, PubSubOption.sendAll(true))
          publishers[key] = publisher
        }

        when (newValue.type) {
          LoggableType.Raw -> publisher!!.setRaw(newValue.raw, table.timestamp)
          LoggableType.Boolean -> publisher!!.setBoolean(newValue.boolean, table.timestamp)
          LoggableType.BooleanArray ->
            publisher!!.setBooleanArray(newValue.booleanArray, table.timestamp)

          LoggableType.Integer -> publisher!!.setInteger(newValue.integer, table.timestamp)
          LoggableType.IntegerArray ->
            publisher!!.setIntegerArray(newValue.integerArray, table.timestamp)

          LoggableType.Float -> publisher!!.setFloat(newValue.float, table.timestamp)
          LoggableType.FloatArray -> publisher!!.setFloatArray(newValue.floatArray, table.timestamp)
          LoggableType.Double -> publisher!!.setDouble(newValue.double, table.timestamp)
          LoggableType.DoubleArray ->
            publisher!!.setDoubleArray(newValue.doubleArray, table.timestamp)

          LoggableType.String -> publisher!!.setString(newValue.string, table.timestamp)
          LoggableType.StringArray ->
            publisher!!.setStringArray(newValue.stringArray, table.timestamp)
        }
      }

      // Update last table
      lastTable = table
    }
  }
}
