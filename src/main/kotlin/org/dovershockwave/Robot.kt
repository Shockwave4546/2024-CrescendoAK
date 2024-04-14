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

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.dovershockwave.subsystem.swerve.commands.SwerveDriveCommand
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher

object Robot : LoggedRobot() {
  override fun robotInit() {
    Logger.recordMetadata("ProjectName", "OmNom")

    when (GlobalConstants.ROBOT_TYPE) {
      RobotType.REAL -> {
        Logger.addDataReceiver(NT4Publisher())
        PowerDistribution() // Enables power distribution logging.
      }

      RobotType.SIM -> {
        Logger.addDataReceiver(NT4Publisher())
      }
    }

    Logger.start()
    RobotContainer

    CommandScheduler.getInstance().onCommandInitialize { command ->
      Logger.recordOutput("/ActiveCommands/${command.name}", true)
    }

    CommandScheduler.getInstance().onCommandFinish { command ->
      Logger.recordOutput("/ActiveCommands/${command.name}", false)
    }

    CommandScheduler.getInstance().onCommandInterrupt { command ->
      Logger.recordOutput("/ActiveCommands/${command.name}", false)
    }
  }


  override fun robotPeriodic() {
    CommandScheduler.getInstance().run()
  }

  override fun disabledInit() {

  }

  override fun disabledPeriodic() {

  }

  override fun autonomousInit() {

  }

  override fun autonomousPeriodic() {

  }

  override fun teleopInit() {
    RobotContainer.swerve.defaultCommand = SwerveDriveCommand(
      RobotContainer.driverController,
      RobotContainer.swerve
    )
  }

  override fun teleopPeriodic() {

  }

  override fun simulationInit() {

  }

  override fun simulationPeriodic() {

  }
}