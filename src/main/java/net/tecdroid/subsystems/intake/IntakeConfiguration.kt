package net.tecdroid.subsystems.intake

import edu.wpi.first.units.measure.Current
import net.tecdroid.util.amps
import net.tecdroid.util.*

data class IntakeConfig(
    val algaeMotorControllerId: NumericId,
    val algaeMotorDirection: RotationalDirection,

    val coralRightMotorControllerId: NumericId,
    val coralLeftMotorControllerId: NumericId,
    val coralMotorsDirection: RotationalDirection,

    val motorsCurrentLimit: Current,
)

public val intakeConfig = IntakeConfig(
    algaeMotorControllerId = NumericId(0),
    algaeMotorDirection = RotationalDirection.Counterclockwise,

    coralRightMotorControllerId = NumericId(0),
    coralLeftMotorControllerId = NumericId(0),
    coralMotorsDirection = RotationalDirection.Clockwise,

    motorsCurrentLimit = 30.0.amps,
)
