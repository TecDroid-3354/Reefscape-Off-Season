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
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import net.tecdroid.subsystems.util.generic.TdSubsystem
import net.tecdroid.util.inches
import net.tecdroid.util.volts

class Intake(private val coralSensors: List<CANrange>) : TdSubsystem("Intake") {
    private val config = intakeConfig
    private val algaeMotorController = TalonFX(config.algaeMotorControllerId.id)
    private val coralRightMotorController = TalonFX(config.coralRightMotorControllerId.id)
    private val coralLeftMotorController = TalonFX(config.coralLeftMotorControllerId.id)
    private val coralTrigger = Trigger { hasCoral() }

    override val forwardsRunningCondition = { true }
    override val backwardsRunningCondition = { true }

    override val motorPosition: Angle
        get() = algaeMotorController.position.value

    override val motorVelocity: AngularVelocity
        get() = algaeMotorController.velocity.value

    override val power: Double
        get() = algaeMotorController.get()

    init {
        require( coralSensors.size == 3 )
        configureMotorInterface()
        configureCanRangesInterface()

        coralTrigger.and { DriverStation.isTeleop() }
            .onTrue(InstantCommand({ setCoralVoltage(0.0.volts) }))
    }

    /**
     * Sets voltage to the CORAL roller motors dynamically based on lateral CANRange sensors.
     */
    fun setCoralVoltage(baseVoltage: Voltage) {
        var leftVoltage = baseVoltage
        var rightVoltage = baseVoltage

        val leftSensor = coralSensors[0]
        val centerSensor = coralSensors[1]
        val rightSensor = coralSensors[2]

        when {
            centerSensor.isDetected.value -> {}
            leftSensor.isDetected.value -> { leftVoltage += 2.0.volts }
            rightSensor.isDetected.value -> { rightVoltage += 2.0.volts }
        }

        coralLeftMotorController.setControl(VoltageOut(leftVoltage))
        coralRightMotorController.setControl(VoltageOut(rightVoltage))
    }


    /**
     * Sets voltage to the ALGAE rollers motor.
     */
    fun setAlgaeVoltage(voltage: Voltage) {
        setVoltage(voltage)
    }

    /**
     * Sets voltage to the ALGAE rollers only.
     */
    override fun setVoltage(voltage: Voltage) {
        algaeMotorController.setControl(VoltageOut(voltage))
    }

    fun hasCoral(): Boolean {
        for (sensor in coralSensors) {
            if (sensor.isDetected.value) return true
        }
        return false
    }

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
