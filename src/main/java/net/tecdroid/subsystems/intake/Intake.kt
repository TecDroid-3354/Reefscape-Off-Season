package net.tecdroid.subsystems.intake

import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.UpdateModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import net.tecdroid.subsystems.util.generic.TdSubsystem
import net.tecdroid.util.inches
import net.tecdroid.util.volts

class Intake(private val coralSensors : List<CANrange>) : TdSubsystem("Intake") {
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
        configureMotorInterface()
        configureCanRangesInterface()
        (coralTrigger).and { DriverStation.isTeleop() }.onTrue(InstantCommand({ setCoralVoltage(0.0.volts) }))
    }

    /**
     * Sets voltage to the CORAL roller motors.
     */
    fun setCoralVoltage(voltage: Voltage) {
        val request = VoltageOut(voltage)
        coralRightMotorController.setControl(request)
    }

    /**
     * Sets voltage to the ALGAE rollers motor.
     * Same as [setVoltage], another function for the mere convenience of the name.
     */
    fun setAlgaeVoltage(voltage: Voltage) {
        setVoltage(voltage)
    }

    /**
     * Sets voltage to the ALGAE rollers, NOT CORALS.
     */
    override fun setVoltage(voltage: Voltage) {
        val request = VoltageOut(voltage)
        algaeMotorController.setControl(request)
    }

    fun hasCoral(): Boolean {
        for (sensor in coralSensors) {
            if (sensor.isDetected.value) return true
        }
        return false
    }

    /**
     * Configures motors for both Coral & Algae Rollers.
     * Sets the follower request for the left coral motor (right coral motor as lead).
     */
    private fun configureMotorInterface() {
        val talonConfig = TalonFXConfiguration()

        with(talonConfig) {
            MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)

            CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(config.motorsCurrentLimit)
        }


        algaeMotorController.clearStickyFaults()
        algaeMotorController.configurator.apply(talonConfig)
        coralRightMotorController.configurator.apply(talonConfig)
        coralLeftMotorController.configurator.apply(talonConfig)

        coralLeftMotorController.setControl(Follower(coralRightMotorController.deviceID, true))
    }

    /**
     * Configures each CANRange taking into account the [com.ctre.phoenix6.configs.ProximityParamsConfigs]
     * and [com.ctre.phoenix6.configs.ToFParamsConfigs]. [com.ctre.phoenix6.configs.FovParamsConfigs] are
     * not taken into account since the object to detect is significantly bigger than the sensor, they should
     * not have problems with their field of view.
     */
    fun configureCanRangesInterface() {
        val canRangeConfig = CANrangeConfiguration()

        with (canRangeConfig) {
            ProximityParams
                .withProximityThreshold(3.5.inches)
                .withProximityHysteresis(0.2.inches)

            ToFParams
                .withUpdateMode(UpdateModeValue.ShortRange100Hz)
        }

        for (sensor in coralSensors) {
            sensor.clearStickyFaults()
            sensor.configurator.apply(canRangeConfig)
        }
    }
}
