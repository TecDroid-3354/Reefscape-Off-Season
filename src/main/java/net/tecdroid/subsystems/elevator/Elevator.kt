package net.tecdroid.subsystems.elevator

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.*
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.subsystems.util.generic.*

class Elevator :
    TdSubsystem("Elevator"),
    MeasurableSubsystem,
    LinearSubsystem,
    LoggableSubsystem,
    VoltageControlledSubsystem
{
    private val config = elevatorConfig
    private val leadMotorController = TalonFX(config.leadMotorControllerId.id)
    private val followerMotorController = TalonFX(config.followerMotorId.id)
    private var target: Angle

    override val forwardsRunningCondition = { displacement < config.measureLimits.relativeMaximum }
    override val backwardsRunningCondition = { displacement > config.measureLimits.relativeMinimum }

    init {
        configureMotorsInterface()
        publishToShuffleboard()
        target = motorPosition
    }

    override fun setVoltage(voltage: Voltage) {
        val request = VoltageOut(voltage)
        leadMotorController.setControl(request)
    }

    override fun setDisplacement(targetDisplacement: Distance) {
        val clampedDisplacement = config.measureLimits.coerceIn(targetDisplacement) as Distance
        val targetAngle = config.sprocket.linearDisplacementToAngularDisplacement(clampedDisplacement)
        val transformedAngle = config.reduction.unapply(targetAngle)
        val request = MotionMagicVoltage(transformedAngle)

        target = transformedAngle
        leadMotorController.setControl(request)
    }

    fun getPositionError(): Angle =
        if (target > motorPosition) target - motorPosition else motorPosition - target

    override val power: Double
        get() = leadMotorController.get()

    override val motorPosition: Angle
        get() = leadMotorController.position.value

    override val motorVelocity: AngularVelocity
        get() = leadMotorController.velocity.value

    override val displacement: Distance
        get() = config.sprocket.angularDisplacementToLinearDisplacement(config.reduction.apply(motorPosition))

    val x = {displacement}

    override val velocity: LinearVelocity
        get() = config.sprocket.angularVelocityToLinearVelocity(config.reduction.apply(motorVelocity))

    fun coast(): Command = Commands.runOnce({
        leadMotorController.setNeutralMode(NeutralModeValue.Coast)
        followerMotorController.setNeutralMode(NeutralModeValue.Coast)
    })

    fun brake(): Command = Commands.runOnce({
        leadMotorController.setNeutralMode(NeutralModeValue.Brake)
        followerMotorController.setNeutralMode(NeutralModeValue.Brake)
    })

    private fun configureMotorsInterface() {
        val talonConfig = TalonFXConfiguration()

        with(talonConfig) {
            MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(config.motorDirection.toInvertedValue())

            CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(config.motorCurrentLimit)

            Slot0
                .withKP(config.controlGains.p)
                .withKI(config.controlGains.i)
                .withKD(config.controlGains.d)
                .withKS(config.controlGains.s)
                .withKV(config.controlGains.v)
                .withKA(config.controlGains.a)
                .withKG(config.controlGains.g)

            MotionMagic
                .withMotionMagicCruiseVelocity(config.reduction.unapply(config.motionTargets.angularVelocity(config.sprocket)))
                .withMotionMagicAcceleration(config.reduction.unapply(config.motionTargets.angularAcceleration(config.sprocket)))
                .withMotionMagicJerk(config.reduction.unapply(config.motionTargets.angularJerk(config.sprocket)))
        }


        leadMotorController.clearStickyFaults()
        followerMotorController.clearStickyFaults()

        leadMotorController.configurator.apply(talonConfig)
        followerMotorController.configurator.apply(talonConfig)

        followerMotorController.setControl(Follower(leadMotorController.deviceID, true))
    }

    override fun initSendable(builder: SendableBuilder) {
        with(builder) {
            addDoubleProperty("Current Displacement (Meters)", { displacement.`in`(Meters) }, {})
            addDoubleProperty("Inverse Operation (Rotations)", { motorPosition.`in`(Rotations) }, {})
        }
    }
}

