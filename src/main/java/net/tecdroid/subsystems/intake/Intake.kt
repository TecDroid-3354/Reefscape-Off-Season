package net.tecdroid.subsystems.intake

import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.UpdateModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import net.tecdroid.subsystems.util.generic.TdSubsystem
import net.tecdroid.util.amps
import net.tecdroid.util.inches
import net.tecdroid.util.volts

/** Intake Subsystem. Please look for the functions with specific names (Coral or Algae).
 * [coralSensors] MUST have the following order:
 * Left Intake CANRange, Center Intake CANRange, Right Intake CANRange, Inner Intake CANRange */
class Intake(private val coralSensors: List<CANrange>) : TdSubsystem("Intake") {
    private val config = intakeConfig
    private val algaeMotorController = TalonFX(config.algaeMotorControllerId.id)
    private val coralRightMotorController = TalonFX(config.coralRightMotorControllerId.id)
    private val coralLeftMotorController = TalonFX(config.coralLeftMotorControllerId.id)
    private val intakingCoralCanRanges = listOf<CANrange>(
        coralSensors[0], coralSensors[1], coralSensors[2]
    )
    private val hasCoralCanRange = coralSensors[3] // 4th CANRange, inside the intake
    // Above this threshold, the motor is considered to be forced (algae already inside)
    private val algaeMotorAmpsThreshold = 20.0.amps

    private val hasCoralTrigger = Trigger { hasCoral() }
    private val intakingCoralTrigger = Trigger { intakingCoral() } // 1st - 3rd CANRange, before fully inside
    private val hasAlgaeTrigger = Trigger { algaeMotorController.supplyCurrent.value > algaeMotorAmpsThreshold }

    override val forwardsRunningCondition = { true }
    override val backwardsRunningCondition = { true }

    override val motorPosition: Angle
        get() = algaeMotorController.position.value

    override val motorVelocity: AngularVelocity
        get() = algaeMotorController.velocity.value

    override val power: Double
        get() = algaeMotorController.get()

    init {
        require( coralSensors.size == 4 ) { "Must have 4 coralSensors" }
        configureMotorInterface()
        configureCanRangesInterface()

        intakingCoralTrigger.and { DriverStation.isTeleop() }
            .onTrue(InstantCommand({ setCoralVoltage(calculateCoralVoltage(8.0.volts)) }))

        hasCoralTrigger.and { DriverStation.isTeleop() }
            .onTrue(InstantCommand({ setCoralVoltage(0.0.volts) }))

        hasAlgaeTrigger.and { DriverStation.isTeleop() }
            .onTrue(InstantCommand({ setAlgaeVoltage(2.0.volts) }))
    }

    /**
     * Sets voltage to the CORAL roller motors.
     * @param coralMotorsVoltage Voltage to be applied to both motors.
     */
    fun setCoralVoltage(coralMotorsVoltage: Voltage) {
        coralLeftMotorController.setControl(VoltageOut(coralMotorsVoltage))
        coralRightMotorController.setControl(VoltageOut(coralMotorsVoltage))
    }

    /**
     * Sets voltage to the CORAL roller motors.
     * @param coralMotorsVoltage A [Pair] containing the desired voltage for each motor, from left to right.
     */
    private fun setCoralVoltage(coralMotorsVoltage: Pair<Voltage, Voltage>) {
        coralLeftMotorController.setControl(VoltageOut(coralMotorsVoltage.first))
        coralRightMotorController.setControl(VoltageOut(coralMotorsVoltage.second))
    }

    fun setCoralVoltageCommand(coralMotorsVoltage: Voltage): Command {
        return InstantCommand({ setCoralVoltage(coralMotorsVoltage) })
    }

    /** This function is necessary for the coral to enter the intake smoothly and not get stuck.
     * When the left CANRange detects the coral we perform leftVoltage += 2.0.volts.
     * When the right CANRange detects the coral we perform rightVoltage += 2.0.volts
     * @return A [Pair] in the following order: coralLeftMotorVoltage, coralRightMotorVoltage.*/
    private fun calculateCoralVoltage(baseVoltage: Voltage) : Pair<Voltage, Voltage> {
        var leftVoltage = baseVoltage
        var rightVoltage = baseVoltage

        WaitUntilCommand { intakingCoral() }

        when {
            intakingCoralCanRanges[1].isDetected.value -> {}                               // Center CANRange
            intakingCoralCanRanges[0].isDetected.value -> { leftVoltage += 2.0.volts }     // Left CANRange
            intakingCoralCanRanges[2].isDetected.value -> { rightVoltage += 2.0.volts }    // Right CANRange
        }
        return Pair(leftVoltage, rightVoltage)
    }


    /**
     * Sets voltage to the ALGAE rollers motor.
     */
    fun setAlgaeVoltage(voltage: Voltage) {
        setVoltage(voltage)
    }

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
        for (sensor in intakingCoralCanRanges) {
            if (sensor.isDetected.value) return true
        }
        return false
    }

    /** Checks the inner CANRange, as it's the one that detects the coral when fully inside intake */
    fun hasCoral(): Boolean = hasCoralCanRange.isDetected.value
    /**
     * Configures motors for both Coral & Algae Rollers independently.
     */
    private fun configureMotorInterface() {
        val talonConfig = TalonFXConfiguration()

        with(talonConfig) {
            MotorOutput.withNeutralMode(NeutralModeValue.Brake)
            CurrentLimits.withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(config.motorsCurrentLimit)
        }

        algaeMotorController.clearStickyFaults()
        algaeMotorController.configurator.apply(talonConfig)
        coralRightMotorController.configurator.apply(talonConfig)
        coralLeftMotorController.configurator.apply(talonConfig)

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

        for (sensor in coralSensors) {
            sensor.clearStickyFaults()
            sensor.configurator.apply(canRangeConfig)
        }
    }
}
