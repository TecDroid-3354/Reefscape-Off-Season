package net.tecdroid.subsystems.climber

import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import net.tecdroid.mechanical.Reduction
import net.tecdroid.safety.MeasureLimits
import net.tecdroid.util.AngularMotionTargets
import net.tecdroid.util.ControlGains
import net.tecdroid.util.NumericId
import net.tecdroid.util.RotationalDirection
import net.tecdroid.util.RotationalDirection.Counterclockwise
import net.tecdroid.util.amps
import net.tecdroid.util.degrees
import net.tecdroid.util.rotations
import net.tecdroid.util.seconds

data class ClimberConfig(
    val rollersMotorControllerId: NumericId,
    val rollersMotorDirection: RotationalDirection,

    val wristMotorControllerId: NumericId,
    val wristMotorDirection: RotationalDirection,

    val motorsCurrentLimit: Current,

    val absoluteEncoderPort: NumericId,
    val absoluteEncoderIsInverted: Boolean,
    val absoluteEncoderOffset: Angle,

    val reduction: Reduction,
    val measureLimits: MeasureLimits<AngleUnit>,
    val controlGains: ControlGains,
    val motionTargets: AngularMotionTargets,
)

val climberConfig = ClimberConfig(
    rollersMotorControllerId = NumericId(0),
    rollersMotorDirection = Counterclockwise,

    wristMotorControllerId = NumericId(0),
    wristMotorDirection = Counterclockwise,

    motorsCurrentLimit = 0.0.amps,

    absoluteEncoderPort = NumericId(0),
    absoluteEncoderIsInverted = false,
    absoluteEncoderOffset = (0.1511).rotations,

    reduction = Reduction(214.285714),

    measureLimits = MeasureLimits(
        absoluteMinimum = 0.0.rotations,
        relativeMinimum = 0.0.rotations,
        relativeMaximum = 0.0.rotations,
        absoluteMaximum = 0.0.rotations,
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
    )
)
