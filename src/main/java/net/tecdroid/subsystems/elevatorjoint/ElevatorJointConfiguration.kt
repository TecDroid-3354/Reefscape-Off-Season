package net.tecdroid.subsystems.elevatorjoint

import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import net.tecdroid.mechanical.Reduction
import net.tecdroid.safety.MeasureLimits
import net.tecdroid.util.*
import net.tecdroid.util.RotationalDirection.Clockwise
import net.tecdroid.util.RotationalDirection.Counterclockwise
import net.tecdroid.util.amps
import net.tecdroid.util.rotations
import net.tecdroid.util.seconds
import net.tecdroid.wrappers.ThroughBoreBrand

data class ElevatorJointConfig(
    val leadMotorControllerId: NumericId,
    val followerMotorControllerId: NumericId,

    val absoluteEncoderPort: NumericId,
    val absoluteEncoderIsInverted: Boolean,
    val absoluteEncoderOffset: Angle,
    val absoluteEncoderBrand: ThroughBoreBrand,

    val motorDirection: RotationalDirection,
    val motorCurrentLimit: Current,

    val reduction: Reduction,
    val measureLimits: MeasureLimits<AngleUnit>,
    val controlGains: ControlGains,
    val motionTargets: AngularMotionTargets,
    val algaeMotionTargets: AngularMotionTargets,
)

val elevatorJointConfig = ElevatorJointConfig(
    leadMotorControllerId = NumericId(55),
    followerMotorControllerId = NumericId(56),
    motorDirection = Counterclockwise,
    motorCurrentLimit = 40.0.amps,

    absoluteEncoderPort = NumericId(0),
    absoluteEncoderIsInverted = true,
    absoluteEncoderOffset = 0.325.rotations - 0.5.degrees,
    absoluteEncoderBrand = ThroughBoreBrand.WCP,

    reduction = Reduction(360.0),

    measureLimits = MeasureLimits(
        absoluteMinimum = 0.011.rotations,
        relativeMinimum = 0.025.rotations,
        relativeMaximum = 0.268.rotations + 1.5.degrees,
        absoluteMaximum = 0.2682.rotations + 1.5.degrees,
    ),

    controlGains = ControlGains(
        p = 0.8,
        s = 0.16263,
        v = 0.11085,
        a = 0.002245,
        g = 0.01139,
    ),

    motionTargets = AngularMotionTargets(
        cruiseVelocity = 0.277.rotations.per(Second),
        accelerationTimePeriod = 0.25.seconds,
        jerkTimePeriod = 0.1.seconds
    ),

    algaeMotionTargets = AngularMotionTargets(
        cruiseVelocity = 0.1.rotations.per(Second),
        accelerationTimePeriod = 1.0.seconds,
        jerkTimePeriod = 0.3.seconds
    )
)
