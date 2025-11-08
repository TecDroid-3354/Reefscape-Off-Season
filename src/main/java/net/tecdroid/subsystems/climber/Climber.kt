package net.tecdroid.subsystems.climber

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.constants.Constants
import net.tecdroid.subsystems.util.generic.AngularSubsystem
import net.tecdroid.subsystems.util.generic.LoggableSubsystem
import net.tecdroid.subsystems.util.generic.TdSubsystem
import net.tecdroid.subsystems.util.generic.VoltageControlledSubsystem
import net.tecdroid.subsystems.util.generic.WithThroughBoreAbsoluteEncoder
import net.tecdroid.util.hertz
import net.tecdroid.wrappers.ThroughBoreAbsoluteEncoder

class Climber :
    TdSubsystem("Climber"),
    LoggableSubsystem,
    WithThroughBoreAbsoluteEncoder,
    AngularSubsystem,
    VoltageControlledSubsystem  {
    private val config = climberConfig
    private val wristController = TalonFX(config.wristMotorControllerId.id, Constants.ALTERNATE_CANBUS_NAME)
    private val rollersController = TalonFX(config.rollersMotorControllerId.id, Constants.ALTERNATE_CANBUS_NAME)
    private lateinit var target : Angle

    override val absoluteEncoder = ThroughBoreAbsoluteEncoder(
        port = config.absoluteEncoderPort,
        offset = config.absoluteEncoderOffset,
        inverted = config.absoluteEncoderIsInverted,
        brand = config.absoluteEncoderBrand,
        canBusName = ""
    )

    override val forwardsRunningCondition  = { angle < config.measureLimits.relativeMaximum }
    override val backwardsRunningCondition = { angle > config.measureLimits.relativeMinimum }


    init {
        configureMotorInterface()
        matchRelativeEncodersToAbsoluteEncoders()
        publishToShuffleboard()
        target = motorPosition
    }

    /**
     * Same as [setAngle], other name for mere convenience and avoid confusion.
     */
    fun setClimberWristAngle(targetAngle: Angle) {
        setAngle(targetAngle)
    }

    /**
     * Sets the voltage for the Climber's ROLLERS.
     */
    fun setClimberRollersVoltage(voltage: Voltage) {
        val request = VoltageOut(voltage)
        rollersController.setControl(request)
    }

    /**
     * Sets the angle for the Climber's WRIST, by clamping it within limits before applying.
     */
    override fun setAngle(targetAngle: Angle) {
        val clampedAngle = config.measureLimits.coerceIn(targetAngle) as Angle
        print("Clamped angle: ")
        print(clampedAngle.`in`(Degrees))
        print("\n")
        val transformedAngle = config.reduction.apply(clampedAngle)
        print("Transformed angle: ")
        print(transformedAngle.`in`(Degrees))
        val request = MotionMagicVoltage(transformedAngle).withSlot(0)

        target = transformedAngle
        wristController.setControl(request)

        //Trigger(forwardsRunningCondition.invoke().not() || backwardsRunningCondition.invoke().not()).onTrue(setVoltage())
    }

    /**
     * Exact same as [setAngle], given this subsystem does not need two different PID, SVAG slot.
     */
    override fun setAngle(targetAngle: Angle, slot: Int) {
        // Do not need other slot
        setAngle(targetAngle)
    }

    /**
     * This method will give voltage to the Climber's WRIST, not rollers.
     * Just for safety, running conditions are checked inside the method, though it's redundant
     * with the SysId running condition. SysId SHOULD BE THE ONLY PLACE WHERE THIS METHOD IS CALLED.
     * NOT INTENDED TO USE FOR ROBOT CONTROL.
     * NOTE:
     * Positive Voltage: Wrist moving away from the center of the robot.
     * Negative Voltage: Wrist moving towards the center of the robot.
     */
    override fun setVoltage(voltage: Voltage) {
        /*if (angle >= config.measureLimits.relativeMaximum && voltage.lt(0.0.volts)) {
            wristController.stopMotor()
        } else if (angle <= config.measureLimits.relativeMinimum && voltage.gt(0.0.volts)) {
            wristController.stopMotor()
        } else {
            wristController.setControl(VoltageOut(voltage))
        }*/
        wristController.setControl(VoltageOut(voltage))
    }

    /**
     * All variables are derived from the Climber's WRIST, as is the only
     * one that needs logging and a sysID routine.
     */
    override val motorPosition: Angle
        get() = wristController.position.value

    override val motorVelocity: AngularVelocity
        get() = wristController.velocity.value

    override val power: Double
        get() = wristController.get()

    override val angle: Angle
        get() = absoluteAngle

    override val angularVelocity: AngularVelocity
        get() = config.reduction.apply(motorVelocity)

    override fun onMatchRelativeEncodersToAbsoluteEncoders() {
        wristController.setPosition(config.reduction.unapply(absoluteAngle))
    }

    /**
     * Configures both the Climber's wrist & rollers motor controllers.
     */
    private fun configureMotorInterface() {
        val talonConfig = TalonFXConfiguration()

        with(talonConfig) {
            MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)

            CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(config.motorsCurrentLimit)

            Slot0
                .withKP(config.controlGains.p)
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

        val wristTalonConfig = talonConfig
        val rollersTalonConfig = talonConfig

        with(wristTalonConfig) {
            MotorOutput
                .withInverted(config.wristMotorDirection.toInvertedValue())
        }

        with(rollersTalonConfig) {
            MotorOutput
                .withInverted(config.rollersMotorDirection.toInvertedValue())
        }

        with(wristController) {
            position.setUpdateFrequency(25.0.hertz)
            motorVoltage.setUpdateFrequency(25.0.hertz)
            velocity.setUpdateFrequency(25.0.hertz)
            optimizeBusUtilization()
        }


        wristController.clearStickyFaults()
        rollersController.clearStickyFaults()
        wristController.configurator.apply(wristTalonConfig)
        rollersController.configurator.apply(rollersTalonConfig)
    }

    /**
     * Logging the Climber's WRIST values.
     */
    override fun initSendable(builder: SendableBuilder) {
        with(builder) {
            addDoubleProperty("Current Angle (Degrees)", { angle.`in`(Degrees) }, {})
            addDoubleProperty("Current Absolute Angle (Degrees)", { absoluteAngle.`in`(Degrees) }, {})
            addDoubleProperty("Climber rolling motor supply current with 8.0 volts", { rollersController.supplyCurrent.value.`in`(Amps) }, {})
        }
    }

    /**
     * Coast for the Climber's WRIST
     */
    fun coast(): Command = Commands.runOnce({
        wristController.setNeutralMode(NeutralModeValue.Coast)
    })

    /**
     * Brake for the Climber's WRIST
     */
    fun brake(): Command = Commands.runOnce({
        wristController.setNeutralMode(NeutralModeValue.Brake)
    })
}