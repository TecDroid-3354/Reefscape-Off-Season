package net.tecdroid.subsystems.elevator

import edu.wpi.first.units.DistanceUnit
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.Current
import net.tecdroid.mechanical.Reduction
import net.tecdroid.safety.MeasureLimits
import net.tecdroid.util.*
import net.tecdroid.util.RotationalDirection.Clockwise
import net.tecdroid.util.Sprocket
import net.tecdroid.util.amps
import net.tecdroid.util.meters
import net.tecdroid.util.seconds

data class ElevatorConfig(
    val leadMotorControllerId: NumericId,
    val followerMotorId: NumericId,
    val motorDirection: RotationalDirection,
    val motorCurrentLimit: Current,

    val reduction: Reduction,
    val sprocket: Sprocket,

    val measureLimits: MeasureLimits<DistanceUnit>,
    val controlGains: ControlGains,
    val motionTargets: LinearMotionTargets,
)

val elevatorConfig = ElevatorConfig(
    leadMotorControllerId = NumericId(50),
    followerMotorId = NumericId(51),
    motorDirection = Clockwise,
    motorCurrentLimit = 40.0.amps,

    reduction = Reduction(8.9285),
    sprocket = Sprocket.fromRadius(Inches.of(1 + 1.0 / 8.0)),

    measureLimits = MeasureLimits(
        absoluteMinimum = 0.0.meters,
        relativeMinimum = 0.001.meters,
        relativeMaximum = 1.040.meters,
        absoluteMaximum = 1.042.meters,
    ),

    controlGains = ControlGains(
        p = 0.2,
        s = 0.11181,
        v = 0.11468,
        a = 0.0028093,
        g = 0.23924
    ),

    motionTargets = LinearMotionTargets(
        cruiseVelocity = 1.2.meters.per(Second),
        accelerationTimePeriod = 0.1.seconds,
        jerkTimePeriod = 0.1.seconds
    ),
)
