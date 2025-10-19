package net.tecdroid.wrappers

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.DutyCycleEncoder
import net.tecdroid.util.NumericId
import net.tecdroid.util.rotations

/* --------------------------------------------- */
/* --> ABSOLUTE GENERALIZATION FOR WCP & REV <-- */
/* --------------------------------------------- */
enum class ThroughBoreBrand { WCP, REV }

sealed interface ThroughBore { fun getAbsoluteReading() : Angle }

/* For WCP ThroughBore 'By CANcoder'. Is literally a CANcoder */
private class WCPThroughBore(port: NumericId, canBusName: String) : ThroughBore {
    private val encoder = CANcoder(port.id, canBusName)
    override fun getAbsoluteReading(): Angle { return encoder.absolutePosition.value }
}

/* For REV Throughbore. Is a DutyCycleEncoder */
private class REVThroughBore(port: NumericId) : ThroughBore {
    private val encoder = DutyCycleEncoder(port.id)
    override fun getAbsoluteReading(): Angle { return Rotations.of(encoder.get()) }
}

/* ---------------------- */
/* --> Actual Wrapper <-- */
/* ---------------------- */
class ThroughBoreAbsoluteEncoder(port: NumericId, private val offset: Angle, private val inverted: Boolean,
                                 private val brand: ThroughBoreBrand, canBusName: String) {
    private val encoder: ThroughBore = when (brand) {
        ThroughBoreBrand.WCP -> WCPThroughBore(port, canBusName)
        ThroughBoreBrand.REV -> REVThroughBore(port)
    }

    private val reading : Angle
        get() = encoder.getAbsoluteReading()

    val position: Angle
        get() = (if (inverted) invertReading(reading) else reading) - offset

    private fun invertReading(angle: Angle) = (1.0.rotations - angle)
}
