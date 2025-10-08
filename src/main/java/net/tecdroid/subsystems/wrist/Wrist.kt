package net.tecdroid.subsystems.wrist

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import net.tecdroid.subsystems.util.generic.*
import net.tecdroid.subsystems.util.identification.GenericSysIdRoutine
import net.tecdroid.wrappers.ThroughBoreAbsoluteEncoder

class Wrist :
    TdSubsystem("Wrist"),
    LoggableSubsystem,
    WithThroughBoreAbsoluteEncoder,
    AngularSubsystem {
    private val config = wristConfig
    private val motorController = TalonFX(config.motorControllerId.id)
    private var target : Angle

    override val absoluteEncoder = ThroughBoreAbsoluteEncoder(
        port = config.absoluteEncoderPort,
        offset = config.absoluteEncoderOffset,
        inverted = config.absoluteEncoderIsInverted,
        brand = config.absoluteEncoderBrand
    )

    override val forwardsRunningCondition  = { angle < config.measureLimits.relativeMaximum }
    override val backwardsRunningCondition = { angle > config.measureLimits.relativeMinimum }

    val sysId = createIdentificationRoutine()

    init {
        configureMotorInterface()
        matchRelativeEncodersToAbsoluteEncoders()
        publishToShuffleboard()
        target = motorPosition
    }

    override fun setVoltage(voltage: Voltage) {
        val request = VoltageOut(voltage)
        motorController.setControl(request)
    }

    override fun setAngle(targetAngle: Angle) {
        val clampedAngle = config.measureLimits.coerceIn(targetAngle) as Angle
        val transformedAngle = config.reduction.unapply(clampedAngle)
        val request = MotionMagicVoltage(transformedAngle).withSlot(0)

        target = transformedAngle
        motorController.setControl(request)
    }

    override fun setAngle(targetAngle: Angle, slot: Int) {
        val clampedAngle = config.measureLimits.coerceIn(targetAngle) as Angle
        val transformedAngle = config.reduction.unapply(clampedAngle)

        target = transformedAngle

        if (slot == 1) {
            val request = DynamicMotionMagicVoltage(transformedAngle,
                config.reduction.unapply(config.algaeMotionTargets.cruiseVelocity),
                config.reduction.unapply(config.algaeMotionTargets.acceleration),
                config.reduction.unapply(config.algaeMotionTargets.jerk)).withSlot(1)
            motorController.setControl(request)
        } else {
            val request = MotionMagicVoltage(transformedAngle).withSlot(0)
            motorController.setControl(request)
        }
    }

    fun sysIdDynamic(direction: SysIdRoutine.Direction): Command {
        return when (direction) {
            SysIdRoutine.Direction.kForward -> sysId.createTests().dynamicForward
            SysIdRoutine.Direction.kReverse -> sysId.createTests().dynamicBackward
        }
    }

    fun sysIdQuasistatic(direction: SysIdRoutine.Direction): Command {
        return when (direction) {
            SysIdRoutine.Direction.kForward -> sysId.createTests().quasistaticForward
            SysIdRoutine.Direction.kReverse -> sysId.createTests().quasistaticBackward
        }
    }

    fun getPositionError(): Angle =
        if (target > motorPosition) target - motorPosition else motorPosition - target

    override val power: Double
        get() = motorController.get()

    override val motorPosition: Angle
        get() = motorController.position.value

    override val motorVelocity: AngularVelocity
        get() = motorController.velocity.value

    override val angle: Angle
        get() = config.reduction.apply(motorPosition)

    override val angularVelocity: AngularVelocity
        get() = config.reduction.apply(motorVelocity)

    override fun onMatchRelativeEncodersToAbsoluteEncoders() {
        motorController.setPosition(config.reduction.unapply(absoluteAngle))
    }

    private fun configureMotorInterface() {
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

            Slot1
                .withKP(0.05)
                .withKI(config.controlGains.i)
                .withKD(config.controlGains.d)
                .withKS(config.controlGains.s)
                .withKV(config.controlGains.v)
                .withKA(config.controlGains.a)
                .withKG(config.controlGains.g)

            MotionMagic
                .withMotionMagicCruiseVelocity(config.reduction.unapply(config.motionTargets.cruiseVelocity))
                .withMotionMagicAcceleration(config.reduction.unapply(config.motionTargets.acceleration))
                .withMotionMagicJerk(config.reduction.unapply(config.motionTargets.jerk))
        }


        motorController.clearStickyFaults()
        motorController.configurator.apply(talonConfig)
    }

    override fun initSendable(builder: SendableBuilder) {
        with(builder) {
            addDoubleProperty("Current Angle (Degrees)", { angle.`in`(Degrees) }, {})
            addDoubleProperty("Current Absolute Angle (Degrees)", { absoluteAngle.`in`(Degrees) }, {})
        }
    }

    fun coast(): Command = Commands.runOnce({
        motorController.setNeutralMode(NeutralModeValue.Coast)
    })

    fun brake(): Command = Commands.runOnce({
        motorController.setNeutralMode(NeutralModeValue.Brake)
    })

}