package net.tecdroid.subsystems.wrist

import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import net.tecdroid.mechanical.Reduction
import net.tecdroid.util.*
import net.tecdroid.util.RotationalDirection.Clockwise
import net.tecdroid.util.amps
import net.tecdroid.util.rotations
import net.tecdroid.util.seconds
import net.tecdroid.safety.MeasureLimits
import net.tecdroid.util.RotationalDirection.Counterclockwise
import net.tecdroid.wrappers.ThroughBoreBrand

data class WristConfig(
    val motorControllerId: NumericId,
    val motorDirection: RotationalDirection,
    val motorCurrentLimit: Current,

    val absoluteEncoderPort: NumericId,
    val absoluteEncoderIsInverted: Boolean,
    val absoluteEncoderOffset: Angle,
    val absoluteEncoderBrand: ThroughBoreBrand,

    val reduction: Reduction,
    val measureLimits: MeasureLimits<AngleUnit>,
    val controlGains: ControlGains,
    val motionTargets: AngularMotionTargets,
    val algaeMotionTargets: AngularMotionTargets
)

val wristConfig = WristConfig(
    motorControllerId = NumericId(55),
    motorDirection = Counterclockwise,
    motorCurrentLimit = 40.0.amps,

    absoluteEncoderPort = NumericId(56),
    absoluteEncoderIsInverted = false,
    absoluteEncoderOffset = 0.0302734375.rotations - 10.0.degrees,
    absoluteEncoderBrand = ThroughBoreBrand.WCP,

    reduction = Reduction(90.7407),

    measureLimits = MeasureLimits(
        absoluteMinimum = (-65.0).degrees,
        relativeMinimum = (-50.0).degrees,
        relativeMaximum = 115.0.degrees,
        absoluteMaximum = 120.0.degrees,
    ),

    controlGains = ControlGains(
        p = 0.1,
        s = 0.11467,
        v = 0.11121,
        a = 0.0019705,
        g = 0.0039384
    ),

    motionTargets = AngularMotionTargets(
        cruiseVelocity = 0.5.rotations.per(Second),
        accelerationTimePeriod = 0.1.seconds,
        jerkTimePeriod = 0.1.seconds
    ),

    algaeMotionTargets = AngularMotionTargets(
        cruiseVelocity = 0.125.rotations.per(Second),
        accelerationTimePeriod = 0.5.seconds,
        jerkTimePeriod = 0.3.seconds
    )
)