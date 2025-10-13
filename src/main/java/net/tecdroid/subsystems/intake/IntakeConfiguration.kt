package net.tecdroid.subsystems.intake

import com.ctre.phoenix6.hardware.CANrange
import edu.wpi.first.units.measure.Current
import net.tecdroid.util.amps
import net.tecdroid.util.*

/** @param intakeOuterCanRanges Goes from left to right */
data class IntakeConfig(
    val algaeMotorControllerId: NumericId,
    val algaeMotorDirection: RotationalDirection,

    val coralRightMotorControllerId: NumericId,
    val coralLeftMotorControllerId: NumericId,
    val coralMotorsDirection: RotationalDirection,

    val motorsCurrentLimit: Current,
    val algaeSupplyCurrentThreshold: Current,

    val intakeLeftCanRange: CANrange,
    val intakeCenterCanRange: CANrange,
    val intakeRightCanRange: CANrange,
    val intakeInnerCanRange: CANrange
)

val intakeConfig = IntakeConfig(
    algaeMotorControllerId = NumericId(59),
    algaeMotorDirection = RotationalDirection.Clockwise,

    coralRightMotorControllerId = NumericId(58),
    coralLeftMotorControllerId = NumericId(57),
    coralMotorsDirection = RotationalDirection.Counterclockwise,

    motorsCurrentLimit = 40.0.amps,
    algaeSupplyCurrentThreshold = 35.0.amps,

    intakeLeftCanRange = CANrange(46),
    intakeCenterCanRange = CANrange(47),
    intakeRightCanRange = CANrange(48),
    intakeInnerCanRange = CANrange(49)
)
