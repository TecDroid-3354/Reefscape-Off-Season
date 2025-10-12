@file:Suppress("MemberVisibilityCanBePrivate")

package net.tecdroid.util

import edu.wpi.first.units.DistanceUnit
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Mult
import kotlin.math.sqrt

//
// Circular
//

class Circle private constructor(val radius: Distance) {

    val diameter: Distance = radius.times(2.0)
    val circumference: Distance = diameter.times(Math.PI)

    fun angularDisplacementToLinearDisplacement(angle: Angle): Distance = circumference.times(angle.`in`(Units.Rotations))
    fun angularVelocityToLinearVelocity(angularVelocity: AngularVelocity): LinearVelocity = circumference.times(angularVelocity.`in`(
        Units.RotationsPerSecond
    )).per(Units.Second)

    fun linearDisplacementToAngularDisplacement(distance: Distance): Angle = Units.Rotations.of(distance.div(circumference).baseUnitMagnitude())
    fun linearVelocityToAngularVelocity(linearVelocity: LinearVelocity): AngularVelocity = Units.Rotations.of(linearVelocity.div(circumference).baseUnitMagnitude()).per(
        Units.Second
    )

    companion object {
        fun fromRadius(radius: Distance) = Circle(radius)
    }
}
typealias Sprocket = Circle
typealias Shaft = Circle

//
// Rectangular
//

open class Rectangle(val length: Distance, val width: Distance) {
    fun scale(scalar: Double) = Rectangle(length * scalar, width * scalar)

    val diagonalLengthSquared: Mult<DistanceUnit, DistanceUnit> = width * width + length * length
    val diagonalLength: Distance = Units.Meters.baseUnit.of(sqrt(diagonalLengthSquared.baseUnitMagnitude()))

}

class Square(sideLength: Distance) : Rectangle(sideLength, sideLength)