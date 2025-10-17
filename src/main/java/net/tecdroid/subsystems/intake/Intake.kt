package net.tecdroid.subsystems.intake

import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.UpdateModeValue
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import net.tecdroid.subsystems.util.generic.LoggableSubsystem
import net.tecdroid.subsystems.util.generic.TdSubsystem
import net.tecdroid.util.inches
import net.tecdroid.util.seconds
import net.tecdroid.util.volts
import java.util.function.BooleanSupplier
import kotlin.math.max

/** Intake Subsystem. Please look for the functions with specific names (Coral or Algae).
 * [coralSensors] MUST have the following order:
 * Left Intake CANRange, Center Intake CANRange, Right Intake CANRange, Inner Intake CANRange */
class Intake(isClimbStateActive: BooleanSupplier) : TdSubsystem("Intake"), LoggableSubsystem {
    private val config = intakeConfig
    private val algaeMotorController = TalonFX(config.algaeMotorControllerId.id)
    private val coralRightMotorController = TalonFX(config.coralRightMotorControllerId.id)
    private val coralLeftMotorController = TalonFX(config.coralLeftMotorControllerId.id)


    private val isClimbState = Trigger { isClimbStateActive.asBoolean }
    private val isHorizontallyDetected = Trigger { isHorizontallyDetected() }
    private val hasCoralTrigger = Trigger { hasCoral() }
    private val intakingCoralTrigger = Trigger { intakingCoral() } // 1st - 3rd CANRange, before fully inside
    private val hasAlgaeTrigger = Trigger {
        algaeMotorController.supplyCurrent.value > config.algaeSupplyCurrentThreshold
    }
    val intakeCanRanges = listOf<CANrange>(config.intakeLeftCanRange, config.intakeCenterCanRange, config.intakeRightCanRange)


    override val forwardsRunningCondition = { true }
    override val backwardsRunningCondition = { true }

    override val motorPosition: Angle
        get() = algaeMotorController.position.value

    override val motorVelocity: AngularVelocity
        get() = algaeMotorController.velocity.value

    override val power: Double
        get() = algaeMotorController.get()

    init {
        configureMotorInterface()
        configureCanRangesInterface()


        isClimbState.whileTrue(SequentialCommandGroup(
            Commands.run({ setCoralVoltage(0.0.volts) }),
            Commands.run({ setAlgaeVoltage(0.0.volts) })
        ))
        isHorizontallyDetected.and { DriverStation.isTeleop() }
            .onTrue(SequentialCommandGroup(
                InstantCommand({ horizontalIntake(Pair(8.0.volts, 10.0.volts)) } ),
                setAlgaeVoltageCommand(0.0.volts)
                ))

        intakingCoralTrigger.and { DriverStation.isTeleop() }.and { hasCoralTrigger.asBoolean.not() }.and { isHorizontallyDetected().not() }
            .onTrue(InstantCommand({ sideIntake(6.0.volts) }))

        hasCoralTrigger.and { DriverStation.isTeleop() }.onTrue(
            WaitCommand(0.1.seconds).andThen(
                ParallelCommandGroup(setCoralVoltageCommand(0.0.volts), setAlgaeVoltageCommand(0.0.volts))
            ))

        hasAlgaeTrigger.and { DriverStation.isTeleop() }
            .onTrue(InstantCommand({ setAlgaeVoltage(1.0.volts) }))
    }

    /**
     * Sets voltage to the coral AND algae roller motors. Algae motor is also necessary as it helps align the coral.
     * @param coralMotorsVoltage Voltage to be applied to all motors.
     */
    fun setCoralVoltage(coralMotorsVoltage: Voltage) {
        coralLeftMotorController.setControl(VoltageOut(coralMotorsVoltage))
        coralRightMotorController.setControl(VoltageOut(-coralMotorsVoltage))
    }

    /**
     * Sets voltage to the coral roller motors.
     * @param coralMotorsVoltage A [Pair] containing the desired voltage for each motor, from left to right.
     */
    private fun setCoralVoltage(coralMotorsVoltage: Pair<Voltage, Voltage>) {
        coralLeftMotorController.setControl(VoltageOut(coralMotorsVoltage.first))
        coralRightMotorController.setControl(VoltageOut(-coralMotorsVoltage.second))
    }

    /**
     * Sets voltage to the coral AND algae roller motors. Algae motor is also necessary as it helps align the coral.
     * @param coralMotorsVoltage Voltage to be applied to all motors.
     */
    fun setCoralVoltageCommand(coralMotorsVoltage: Voltage): Command {
        return InstantCommand({ setCoralVoltage(coralMotorsVoltage) })
    }

    /** This function is necessary for the coral to enter the intake smoothly and not get stuck.
     * When the left CANRange detects the coral we perform leftVoltage += 4.0.volts.
     * When the right CANRange detects the coral we perform rightVoltage += 4.0.volts
     * @return A [Pair] in the following order: coralLeftMotorVoltage, coralRightMotorVoltage.*/
    fun sideIntake(baseVoltage: Voltage) {
        var leftVoltage = baseVoltage
        var rightVoltage = baseVoltage

        if (config.intakeLeftCanRange.isDetected.value) rightVoltage += 4.0.volts
        else if (config.intakeRightCanRange.isDetected.value) leftVoltage += 4.0.volts

        SequentialCommandGroup(
            InstantCommand ({ setCoralVoltage(Pair(leftVoltage, rightVoltage.unaryMinus())) }).andThen(setAlgaeVoltageCommand(12.0.volts)),
            WaitCommand(0.1.seconds).andThen({ setCoralVoltage(Pair(leftVoltage, rightVoltage)) })
        )
    }

    // coral voltage
    private fun horizontalIntake(voltage: Pair<Voltage, Voltage>) {
        ParallelCommandGroup (
            InstantCommand({ setCoralVoltage(Pair(voltage.first, voltage.second.unaryMinus())) })
                .andThen(setAlgaeVoltageCommand(12.0.volts)), // second would invert twice on purpose
            WaitUntilCommand { config.intakeLeftCanRange.isDetected.value.not() }
                .andThen({ setCoralVoltage(voltage) })
        )
    }


    /**
     * Sets voltage to the ALGAE rollers' motor.
     */
    fun setAlgaeVoltage(voltage: Voltage) {
        setVoltage(voltage)
    }

    /**
     * Sets voltage to the ALGAE rollers' motor.
     */
    fun setAlgaeVoltageCommand(voltage: Voltage): Command {
        return InstantCommand({ setAlgaeVoltage(voltage) })
    }

    /**
     * Sets voltage to the ALGAE rollers only.
     */
    override fun setVoltage(voltage: Voltage) {
        algaeMotorController.setControl(VoltageOut(voltage))
    }

    private fun intakingCoral() : Boolean {
        // Basically a for each.
        for (sensor in listOf<CANrange>(config.intakeLeftCanRange, config.intakeCenterCanRange, config.intakeRightCanRange)) {
            if (sensor.isDetected.value) return true
        }
        return false
    }

    /** Checks the inner CANRange adn the center one to ensure the coral has aligned correctly, as it's the one that detects the coral when fully inside intake */
    fun hasCoral(): Boolean = config.intakeInnerCanRange.isDetected.value && config.intakeCenterCanRange.isDetected.value

    fun isHorizontallyDetected(): Boolean {
        for (sensor in listOf<CANrange>(config.intakeLeftCanRange, config.intakeCenterCanRange, config.intakeRightCanRange)) {
            if (sensor.isDetected.value.not()) return false
        }
        return true
    }

    /**
     * Configures motors for both Coral & Algae Rollers independently.
     */
    private fun configureMotorInterface() {
        val talonConfig = TalonFXConfiguration()

        with(talonConfig) {
            MotorOutput.withNeutralMode(NeutralModeValue.Brake)
                .withInverted(config.coralMotorsDirection.toInvertedValue())
            CurrentLimits.withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(config.motorsCurrentLimit)

        }

        algaeMotorController.clearStickyFaults()
        coralRightMotorController.clearStickyFaults()
        coralLeftMotorController.clearStickyFaults()
        coralRightMotorController.configurator.apply(talonConfig)
        coralLeftMotorController.configurator.apply(talonConfig)
        algaeMotorController.configurator.apply(talonConfig.MotorOutput.withInverted(
            config.algaeMotorDirection.toInvertedValue()
        ))

        // No follower: cada motor puede recibir voltaje distinto
    }

    /**
     * Configures CANRange sensors.
     */
    fun configureCanRangesInterface() {
        val canRangeConfig = CANrangeConfiguration()

        with(canRangeConfig) {
            ProximityParams.withProximityThreshold(3.5.inches)
                .withProximityHysteresis(0.2.inches)

            ToFParams.withUpdateMode(UpdateModeValue.ShortRange100Hz)

        }

        for (sensor in listOf<CANrange>(
            config.intakeLeftCanRange, config.intakeCenterCanRange, config.intakeRightCanRange, config.intakeInnerCanRange)) {
            sensor.clearStickyFaults()
            sensor.configurator.apply(canRangeConfig)
        }
    }

    override fun initSendable(builder: SendableBuilder) {
        with (builder) {
            addDoubleProperty("Intake algae motor amperage with 12V ", { algaeMotorController.supplyCurrent.value.`in`(Amps) }, {})
        }
    }
}
