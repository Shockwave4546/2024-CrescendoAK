package org.dovershockwave.subsystem.pose

import edu.wpi.first.math.geometry.Rotation2d
import java.util.*

data class DistanceAnglePair(val distance: Optional<Double>, val rotation2d: Optional<Rotation2d>)
