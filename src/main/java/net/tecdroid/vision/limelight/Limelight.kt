@file:Suppress("MemberVisibilityCanBePrivate", "SameParameterValue", "unused")

package net.tecdroid.vision.limelight

import edu.wpi.first.math.Num
import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Frequency
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.util.Color
import frc.robot.LimelightHelpers
import net.tecdroid.util.*

/**
 * Represents [Limelight] configuration parameters
 *
 * @param name The name of the limelight
 * @param cameraPositionInRobotSpace The limelight's offset from the center of the robot
 *
 * **Definition of Robot Space**
 * - 3d Cartesian Coordinate System with (0,0,0) located at the center of the robot’s frame projected down to the floor. (Assuming you set the correct limelight offsets)
 * - X+ → Pointing forward (Forward Vector)
 * - Y+ → Pointing toward the robot’s right (Right Vector)
 * - Z+ → Pointing upward (Up Vector)
 */
data class LimelightConfig(
    val name: String,
    val cameraPositionInRobotSpace: Pose3d,
)

/**
 * Base class for declaring a basic [Limelight]
 *
 * @param config The [LimelightConfig] for this limelight
 */
abstract class LimelightBase(val config: LimelightConfig) {
    private val table = NetworkTableInstance.getDefault().getTable(config.name)

    init {
//        setDoubleArray(LimelightTableKeys.Set.cameraPositionInRobotSpace, arrayOf(
//            config.cameraPositionInRobotSpace.measureX.`in`(Meters),
//            config.cameraPositionInRobotSpace.measureY.`in`(Meters),
//            config.cameraPositionInRobotSpace.measureZ.`in`(Meters),
//            config.cameraPositionInRobotSpace.rotation.measureX.`in`(Degrees),
//            config.cameraPositionInRobotSpace.rotation.measureY.`in`(Degrees),
//            config.cameraPositionInRobotSpace.rotation.measureZ.`in`(Degrees)
//        ).toDoubleArray())
    }

    /**
     * Returns a [NetworkTableEntry] from this [Limelight]'s [NetworkTable]
     */
    private fun getTableEntry(name: String): NetworkTableEntry = table.getEntry(name)

    /**
     * Fetches [name] from the associated [NetworkTable] as a [Double]
     */
    protected fun getDouble(name: String): Double = getTableEntry(name).getDouble(0.0)

    /**
     * Sets [name] in the associated [NetworkTable] to [value] as a [Double]
     */
    protected fun setDouble(name: String, value: Double) = getTableEntry(name).setDouble(value)

    /**
     * Sets [name] in the associated [NetworkTable] to [value] as a [Number]
     */
    protected fun setNumber(name: String, value: Number) = getTableEntry(name).setNumber(value)

    /**
     * Fetches [name] from the associated [NetworkTable] as a [String]
     */
    protected fun getString(name: String): String = getTableEntry(name).getString("")

    /**
     * Sets [name] in the associated [NetworkTable] to [value] as a [String]
     */
    protected fun setString(name: String, value: String) = getTableEntry(name).setString(value)

    /**
     * Fetches [name] from the associated [NetworkTable] as a [DoubleArray]
     */
    protected fun getDoubleArray(name: String): DoubleArray = getTableEntry(name).getDoubleArray(DoubleArray(0))

    /**
     * Sets [name] in the associated [NetworkTable] to [value] as a [DoubleArray]
     */
    protected fun setDoubleArray(name: String, value: DoubleArray) = getTableEntry(name).setDoubleArray(value)

    /**
     * Sets [name] in the associated [NetworkTable] to [value] as an [Array] of [Number]
     */
    protected fun setNumberArray(name: String, value: Array<Number>) = getTableEntry(name).setNumberArray(value)

    /**
     * Fetches [name] from the associated [NetworkTable] as an [Array] of [String]
     */
    protected fun getStringArray(name: String): Array<String> = getTableEntry(name).getStringArray(Array(0) { "" })
}

/**
 * Implements basic [Limelight] functionality. Assumes no pipelines.
 *
 * @param config The [LimelightConfig] for this limelight
 */
open class Limelight(config: LimelightConfig) : LimelightBase(config) {
    //
    // Target Validity
    //

    /**
     * Determines if a valid target is in sight
     */
    val hasTarget: Boolean
        get() = getDouble(LimelightTableKeys.Get.hasValidTarget) == 1.0

    /**
     * Determines if more than one valid target is in sight
     */
    val hasMultipleTargets: Boolean
        get() = targetCount > 1

    /**
     * Returns the amount of targets currently in the camera's view
     */
    val targetCount: Int
        get() = detectionSnapshot.count

    /**
     * Returns a snapshot of all the detection data in the camera's vision
     */
    val detectionSnapshot: LimelightDetection2d
        get() = LimelightDetection2d.fromRawData(getDoubleArray(LimelightTableKeys.Get.targetingDataR2))

    //
    // Basic Vision Data
    //

    /**
     * Obtains the horizontal angular offset from the center of the target
     */
    val horizontalOffset: Angle
        get() = getDouble(LimelightTableKeys.Get.horizontalOffsetDegrees).degrees

    /**
     * Obtains the vertical angular offset from the center of the target
     */
    val verticalOffset: Angle
        get() = getDouble(LimelightTableKeys.Get.verticalOffsetDegrees).degrees

    /**
     * Obtains what percentage of the camera's vision is being occupied by the target
     */
    val targetAreaOccupancy: Percentage
        get() = Percentage(getDouble(LimelightTableKeys.Get.targetAreaOccupancyPercentage))

    //
    // Hardware data
    //

    /**
     * The pipeline's latency
     */
    val pipelineLatency: Time
        get() = getDouble(LimelightTableKeys.Get.pipelineLatency).milliseconds

    /**
     * The capture latency
     */
    val captureLatency: Time
        get() = getDouble(LimelightTableKeys.Get.captureLatency).milliseconds

    /**
     * The total latency
     */
    val latency: Time
        get() = pipelineLatency + captureLatency

    /**
     * The device's heartbeat
     */
    val heartbeat: Double
        get() = getDouble(LimelightTableKeys.Get.heartbeat)

    /**
     * A snapshot of the current hardware state
     */
    val hardwareMetrics: LimelightHardwareMetrics
        get() = LimelightHardwareMetrics.fromRawData(getDoubleArray(LimelightTableKeys.Get.hardwareMetrics))

    //
    // Pipeline Data
    //

    /**
     * The current pipeline index
     */
    var pipelineIndex: Int
        get() = getDouble(LimelightTableKeys.Get.pipelineIndex).toInt()
        set(value) {
            require(value in 0..9) { "Limelight Pipeline index must be anywhere between 0 and 9" }
            setNumber(LimelightTableKeys.Set.pipelineIndex, value.toDouble())
        }

    // TODO: Convert to enumeration
    /**
     * Obtains the name of the current pipeline
     */
    val pipelineTypeName: String
        get() = getString(LimelightTableKeys.Get.pipelineType)

    //
    // Overlay data
    //

    /**
     * The position of both crosshairs
     */
    val crosshairs: LimelightCrosshairs
        get() = LimelightCrosshairs.fromRawData(getDoubleArray(LimelightTableKeys.Get.crosshairPosition))

    /**
     * Returns the average color of the 3x3 pixel grid formed at the crosshair
     */
    val colorAtCrosshair: Color
        get() = rawDataToColor(getDoubleArray(LimelightTableKeys.Get.hsvAtCrosshair))

    //
    // Setters only
    //

    var ledMode: LimelightLedMode = LimelightLedMode.Pipeline
        set(value) {
            setNumber(LimelightTableKeys.Set.ledMode, value.mode.toDouble())
            field = value
        }

    var streamingMode: LimelightStreamMode = LimelightStreamMode.Main
        set(value) {
            setNumber(LimelightTableKeys.Set.streamingMode, value.mode.toDouble())
            field = value
        }

    var crop: LimelightCrop = LimelightCrop(0.0, 0.0, 0.0, 0.0)
        set(value) {
            setNumberArray(LimelightTableKeys.Set.cropParameters, value.array)
            field = value
        }

    var idFilter: Array<Int> = arrayOf()
        set(value) {
            field = value
            setDoubleArray("fiducial_id_filters_set", value.map { it.toDouble() }.toDoubleArray())
        }

    //
    // Uncategorized
    //

    /**
     * JSON output of limelight readings
     */
    val json: String
        get() = getString(LimelightTableKeys.Get.jsonOutput)

    //
    // Raw Stuff
    //

    /**
     * Obtains the horizontal pixel offset from the center of the target
     */
    val rawHorizontalOffset: Pixels
        get() = getDouble(LimelightTableKeys.Get.Raw.horizontalOffsetPixels).toInt().pixels

    /**
     * Obtains the vertical pixel offset from the center of the target
     */
    val rawVerticalOffset: Pixels
        get() = getDouble(LimelightTableKeys.Get.Raw.verticalOffsetPixels).toInt().pixels

    /**
     * Obtains the raw corner data of the detections in sight
     */
    val rawCorners: Array<LimelightRawCornerDetection>
        get() = LimelightRawCornerDetection.fromRawData(getDoubleArray(LimelightTableKeys.Get.Raw.cornerData))

    /**
     * Obtains the raw fiducial data of the detections in sight
     */
    val rawFiducials: Array<LimelightRawFiducials>
        get() = LimelightRawFiducials.fromRawData(getDoubleArray(LimelightTableKeys.Get.Raw.fiducialData))

    /**
     * Obtains the raw detection data of the detections in sight
     */
    val rawDetections: Array<LimelightRawDetection2d>
        get() = LimelightRawDetection2d.fromRawData(getDoubleArray(LimelightTableKeys.Get.Raw.detectionData))

}

/**
 * [Limelight] variant that specializes in AprilTag detection
 *
 * @param config The [LimelightConfig] for this limelight
 */
class LimelightAprilTagDetector(config: LimelightConfig): Limelight(config) {
    //
    // Basic AprilTag Data
    //

    /**
     * Returns the id of the main target in sight
     */
    val targetId: Int
        get() = getDouble(LimelightTableKeys.Get.targetId).toInt()

    //
    // Positions in Field Space
    //

    /**
     * Returns the robot's position with the origin at the middle of the field
     */
    val robotPosition: LimelightDetection3d
        get() = LimelightDetection3d.fromRawData(getDoubleArray(LimelightTableKeys.Get.robotPosition))

    /**
     * Returns the robot's position with the origin at the middle of the field. Uses MegaTag2
     */
    val robotPositionMt2: LimelightDetection3d
        get() = LimelightDetection3d.fromRawData(getDoubleArray(LimelightTableKeys.Get.robotPositionMt2))

    /**
     * Returns the robot's position with the origin at the blue alliance origin
     */
    val robotPositionInBlueFieldSpace: LimelightDetection3d
        get() = LimelightDetection3d.fromRawData(getDoubleArray(LimelightTableKeys.Get.robotPositionInBlueFieldSpace))

    /**
     * Returns the robot's position with the origin at the blue alliance origin. Uses MegaTag2
     */
    val robotPositionInBlueFieldSpaceMt2: LimelightDetection3d
        get() = LimelightDetection3d.fromRawData(getDoubleArray(LimelightTableKeys.Get.robotPositionInBlueFieldSpaceMt2))

    /**
     * Returns the robot's position with the origin at the red alliance origin and rotated 180 degrees
     */
    val robotPositionInRedFieldSpace: LimelightDetection3d
        get() = LimelightDetection3d.fromRawData(getDoubleArray(LimelightTableKeys.Get.robotPositionInRedFieldSpace))

    val robotPoseEstimateWpiBlueMT1: LimelightHelpers.PoseEstimate
        get() = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name)
    /**
     * Returns the robot's position with the origin at the red alliance origin and rotated 180 degrees. Uses MegaTag2
     */
    val robotPositionInRedFieldSpaceMt2: LimelightDetection3d
        get() = LimelightDetection3d.fromRawData(getDoubleArray(LimelightTableKeys.Get.robotPositionInRedFieldSpaceMt2))

    //
    // Positions in Target Space
    //

    /**
     * The robot's position in target space
     *
     * **Definition of Target Space**
     * - 3d Cartesian Coordinate System with (0,0,0) at the center of the target.
     * - X+ → Pointing to the right of the target (If you are looking at the target)
     * - Y+ → Pointing downward
     * - Z+ → Pointing out of the target (orthogonal to target's plane).
     */
    val robotPositionInTargetSpace: Pose3d
        get() = rawDataToPose3d(getDoubleArray(LimelightTableKeys.Get.robotPositionInTargetSpace))

    /**
     * The camera's position in target space
     *
     * **Definition of Target Space**
     * - 3d Cartesian Coordinate System with (0,0,0) at the center of the target.
     * - X+ → Pointing to the right of the target (If you are looking at the target)
     * - Y+ → Pointing downward
     * - Z+ → Pointing out of the target (orthogonal to target's plane).
     */
    val cameraPositionInTargetSpace: Pose3d
        get() = rawDataToPose3d(getDoubleArray(LimelightTableKeys.Get.cameraPositionInTargetSpace))

    //
    // Positions in Robot Space
    //

    /**
     * The target's position in robot space
     *
     * **Definition of Robot Space**
     * - 3d Cartesian Coordinate System with (0,0,0) located at the center of the robot’s frame projected down to the floor. (Assuming you set the correct limelight offsets)
     * - X+ → Pointing forward (Forward Vector)
     * - Y+ → Pointing toward the robot’s right (Right Vector)
     * - Z+ → Pointing upward (Up Vector)
     */
    val targetPositionInRobotSpace: Pose3d
        get() = rawDataToPose3d(getDoubleArray(LimelightTableKeys.Get.targetPositionInRobotSpace))

    /**
     * The camera's position in robot space
     *
     * **Definition of Robot Space**
     * - 3d Cartesian Coordinate System with (0,0,0) located at the center of the robot’s frame projected down to the floor. (Assuming you set the correct limelight offsets)
     * - X+ → Pointing forward (Forward Vector)
     * - Y+ → Pointing toward the robot’s right (Right Vector)
     * - Z+ → Pointing upward (Up Vector)
     */
    var cameraPositionInRobotSpace: Pose3d
        get() = rawDataToPose3d(getDoubleArray(LimelightTableKeys.Get.cameraPositionInRobotSpace))
        set(value) { setDoubleArray(LimelightTableKeys.Set.cameraPositionInRobotSpace, pose3dToRawData(value)) }

    //
    // Positions in Camera Space
    //

    /**
     * The target's position in camera space
     *
     * **Definition of Camera Space**
     * - 3d Cartesian Coordinate System with (0,0,0) at the camera lens.
     * - X+ → Pointing to the right (if you were to embody the camera)
     * - Y+ → Pointing downward
     * - Z+ → Pointing out of the camera
     */
    val targetPositionInCameraSpace: Pose3d
        get() = rawDataToPose3d(getDoubleArray(LimelightTableKeys.Get.targetPositionInCameraSpace))

    //
    // Miscellaneous
    //

    /**
     * The standard deviation for MegaTag detections. Useful for Pose Estimation
     */
    val megaTagStandardDeviations: Double
        get() = getDouble(LimelightTableKeys.Get.megaTagStandardDeviations)

    /**
     * Provides a summary of the camera's current detection metrics
     */
    val detectionState: LimelightDetection2d
        get() = LimelightDetection2d.fromRawData(getDoubleArray(LimelightTableKeys.Get.targetingDataR2))

    var robotOrientation: Pair<Rotation3d, Rotation3d> = Rotation3d() to Rotation3d()
        set(value) {
            field = value
            setDoubleArray(LimelightTableKeys.Set.robotOrientation, orientationToRawData(value.first, value.second))
        }

    fun setThrottle(value: Int) { setNumber(LimelightTableKeys.Set.throttle, value) }
}

/**
 * Base class for [Limelight]s that specialize in detection through the use of neural networks
 *
 * @param config The [LimelightConfig] for this limelight
 */
abstract class LimelightNeuralBase<T>(config: LimelightConfig, protected val computingMethod: (String) -> T) : Limelight(config) {
    /**
     * Primary neural detector or classifier class name
     */
    val generalNeuralClassName: String
        get() = getString(LimelightTableKeys.Get.neuralClassName)

    /**
     * Transforms the targeted detection or classification into its corresponding object
     */
    abstract fun computeResult(classname: String): T
}

/**
 * [Limelight] variant that specializes in classifying objects through a neural network
 *
 * @param config The [LimelightConfig] for this limelight
 */
class LimelightNeuralClassifier<T>(config: LimelightConfig, computingMethod: (String) -> T) : LimelightNeuralBase<T>(config, computingMethod) {
    /**
     * Classifier's computed class name
     */
    private val classifierClassName: String
        get() = getString(LimelightTableKeys.Get.classifierPipelineClassname)

    override fun computeResult(classname: String): T = computingMethod(classifierClassName)
}

/**
 * [Limelight] variant that specializes in detecting objects through a neural network
 *
 * @param config The [LimelightConfig] for this limelight
 */
class LimelightNeuralDetector<T>(config: LimelightConfig, computingMethod: (String) -> T) : LimelightNeuralBase<T>(config, computingMethod) {
    /**
     * Detector's primary detection name
     */
    private val detectorClassName: String
        get() = getString(LimelightTableKeys.Get.detectorPipelineClassname)

    override fun computeResult(classname: String): T = computingMethod(detectorClassName)

}

/**
 * [Limelight] variant that specializes in detecting barcodes and reading their data
 *
 * @param config The [LimelightConfig] for this limelight
 */
class LimelightBarcodeDetector(config: LimelightConfig): Limelight(config) {
    /**
     * Raw barcode data
     */
    val barcodeData: Array<String>
        get() = getStringArray(LimelightTableKeys.Get.Raw.barcodeData)
}

//
// Helpers
//

internal fun rawDataToPose3d(data: DoubleArray) : Pose3d {
    require(data.size >= 6) { "Data must have at least 6 entries" }
    return Pose3d(
        Translation3d(data[0], data[1] , data[2]),
        Rotation3d(data[3], data[4], data[5])
    )
}

internal fun pose3dToRawData(pose: Pose3d) : DoubleArray {
    return arrayOf(pose.x, pose.y, pose.z, pose.rotation.x, pose.rotation.y, pose.rotation.z).toDoubleArray()
}

internal fun rawDataToOrientation(data: DoubleArray): Pair<Rotation3d, Rotation3d> {
    return Rotation3d(data[0], data[2], data[4]) to Rotation3d(data[1], data[3], data[5])
}

internal fun orientationToRawData(rotation3d: Rotation3d, rotationRates3d: Rotation3d): DoubleArray {
    return arrayOf(rotation3d.x, rotationRates3d.x, rotation3d.y, rotationRates3d.y, rotation3d.z, rotationRates3d.z).toDoubleArray()
}

/**
 * Converts the [DoubleArray] of color under the crosshair of a [Limelight] to a [Color] object
 */
internal fun rawDataToColor(data: DoubleArray): Color =
    Color.fromHSV(data[LimelightIndices.Tc.hue].toInt(),
        data[LimelightIndices.Tc.saturation].toInt(),
        data[LimelightIndices.Tc.value].toInt()
    )

/**
 * Contains the data obtained from polling the position of a [Limelight]'s crosshairs
 */
data class LimelightCrosshairs(
    val positionOne: Translation2d,
    val positionTwo: Translation2d
) {
    val hasSingleCrosshair = positionOne == positionTwo

    companion object {
        fun fromRawData(data: DoubleArray) = LimelightCrosshairs(
            positionOne = Translation2d(data[0], data[1]),
            positionTwo = Translation2d(data[2], data[3])
        )
    }
}

data class LimelightRawCornerDetection(
    val first: Translation2d,
    val second: Translation2d,
    val third: Translation2d,
    val fourth: Translation2d
) {
    companion object {
        fun fromRawData(data: DoubleArray) = Array(data.size / LimelightIndices.TCornXY.entriesPerCorner) {
            LimelightRawCornerDetection(
                first = Translation2d(
                    data[LimelightIndices.TCornXY.x0 + (it * LimelightIndices.TCornXY.entriesPerCorner)],
                    data[LimelightIndices.TCornXY.y0 + (it * LimelightIndices.TCornXY.entriesPerCorner)]
                ),
                second = Translation2d(
                    data[LimelightIndices.TCornXY.x1 + (it * LimelightIndices.TCornXY.entriesPerCorner)],
                    data[LimelightIndices.TCornXY.y1 + (it * LimelightIndices.TCornXY.entriesPerCorner)]
                ),
                third = Translation2d(
                    data[LimelightIndices.TCornXY.x2 + (it * LimelightIndices.TCornXY.entriesPerCorner)],
                    data[LimelightIndices.TCornXY.y2 + (it * LimelightIndices.TCornXY.entriesPerCorner)]
                ),
                fourth = Translation2d(
                    data[LimelightIndices.TCornXY.x3 + (it * LimelightIndices.TCornXY.entriesPerCorner)],
                    data[LimelightIndices.TCornXY.y3 + (it * LimelightIndices.TCornXY.entriesPerCorner)]
                )
            )
        }
    }
}

/**
 * Stores a snapshot of raw 2-dimensional target data
 * **From Limelight Documentation:** Enable "send contours" in the "Output" tab to stream corner coordinates
 */
class LimelightRawTarget(
    val normalizedHorizontalOffset: Double,
    val normalizedVerticalOffset: Double,
    val coverage: Percentage
) {
    companion object {
        fun fromRawData(data: DoubleArray) = Array(data.size / LimelightIndices.RawTargets.entriesPerDetection) {
            LimelightRawTarget(
                normalizedHorizontalOffset = data[LimelightIndices.RawTargets.txnc + (it * LimelightIndices.RawTargets.entriesPerDetection)],
                normalizedVerticalOffset = data[LimelightIndices.RawTargets.tync + (it * LimelightIndices.RawTargets.entriesPerDetection)],
                coverage = data[LimelightIndices.RawTargets.ta + (it * LimelightIndices.RawTargets.entriesPerDetection)].percent
            )
        }
    }
}

/**
 * Stores a snapshot of raw 2-dimensional fiducial data
 */
data class LimelightRawFiducials(
    val id: Int,
    val horizontalOffset: Pixels,
    val verticalOffset: Pixels,
    val coverage: Percentage,
    val distanceToCamera: Distance,
    val distanceToRobot: Distance,
    val ambiguity: Double
) {
    companion object {
        fun fromRawData(data: DoubleArray) = Array(data.size / LimelightIndices.RawFiducials.entriesPerDetection) {
            LimelightRawFiducials(
                id = data[LimelightIndices.RawFiducials.id + (it * LimelightIndices.RawFiducials.entriesPerDetection)].toInt(),
                horizontalOffset = data[LimelightIndices.RawFiducials.txnc + (it * LimelightIndices.RawFiducials.entriesPerDetection)].toInt().pixels,
                verticalOffset = data[LimelightIndices.RawFiducials.tync + (it * LimelightIndices.RawFiducials.entriesPerDetection)].toInt().pixels,
                coverage = data[LimelightIndices.RawFiducials.ta + (it * LimelightIndices.RawFiducials.entriesPerDetection)].percent,
                distanceToCamera = data[LimelightIndices.RawFiducials.distToCamera + (it * LimelightIndices.RawFiducials.entriesPerDetection)].meters,
                distanceToRobot = data[LimelightIndices.RawFiducials.distToRobot + (it * LimelightIndices.RawFiducials.entriesPerDetection)].meters,
                ambiguity =  data[LimelightIndices.RawFiducials.ambiguity + (it * LimelightIndices.RawFiducials.entriesPerDetection)],
            )
        }
    }
}

/**
 * Stores a snapshot of raw 2-dimensional detection data
 * **From Limelight Documentation:** Enable "send contours" in the "Output" tab to stream corner coordinates
 */
data class LimelightRawDetection2d(
    val id: Int,
    val horizontalOffset: Pixels,
    val verticalOffset: Pixels,
    val coverage: Percentage,
    val corners: LimelightRawCornerDetection
) {
    companion object {
        fun fromRawData(data: DoubleArray) = Array(data.size / LimelightIndices.RawFiducials.entriesPerDetection) {
            LimelightRawDetection2d(
                id                 = data[LimelightIndices.RawDetections.id + (it * LimelightIndices.RawDetections.entriesPerDetection)].toInt(),
                horizontalOffset = data[LimelightIndices.RawDetections.txnc + (it * LimelightIndices.RawDetections.entriesPerDetection)].toInt().pixels,
                verticalOffset   = data[LimelightIndices.RawDetections.tync + (it * LimelightIndices.RawDetections.entriesPerDetection)].toInt().pixels,
                coverage           = data[LimelightIndices.RawDetections.ta + (it * LimelightIndices.RawDetections.entriesPerDetection)].percent,
                corners = LimelightRawCornerDetection(
                    first = Translation2d(
                        data[LimelightIndices.RawDetections.x0 + (it * LimelightIndices.RawDetections.entriesPerDetection)],
                        data[LimelightIndices.RawDetections.y0 + (it * LimelightIndices.RawDetections.entriesPerDetection)]
                    ),
                    second = Translation2d(
                        data[LimelightIndices.RawDetections.x1 + (it * LimelightIndices.RawDetections.entriesPerDetection)],
                        data[LimelightIndices.RawDetections.y1 + (it * LimelightIndices.RawDetections.entriesPerDetection)]
                    ),
                    third = Translation2d(
                        data[LimelightIndices.RawDetections.x2 + (it * LimelightIndices.RawDetections.entriesPerDetection)],
                        data[LimelightIndices.RawDetections.y2 + (it * LimelightIndices.RawDetections.entriesPerDetection)]
                    ),
                    fourth = Translation2d(
                        data[LimelightIndices.RawDetections.x3 + (it * LimelightIndices.RawDetections.entriesPerDetection)],
                        data[LimelightIndices.RawDetections.y3 + (it * LimelightIndices.RawDetections.entriesPerDetection)]
                    )
                )
            )
        }
    }
}

/**
 * Stores a snapshot of 2-dimensional detection data
 */
data class LimelightDetection2d(
    val isValidData: Boolean = false,
    val hasValidTarget: Boolean = false,
    val count: Int = 0,
    val latency: Time = 0.0.milliseconds,
    val captureLatency: Time = 0.0.milliseconds,
    val horizontalAngularOffset: Angle = 0.0.degrees,
    val verticalAngularOffset: Angle = 0.0.degrees,
    val horizontalLinearOffset: Pixels = 0.pixels,
    val verticalLinearOffset: Pixels = 0.pixels,
    val areaOccupancy: Percentage = 0.0.percent,
    val id: Int = 0,
    val classifierClassIndex: Int = 0,
    val detectorClassIndex: Int = 0,
    val longSidePixels: Pixels = 0.pixels,
    val shortSidePixels: Pixels = 0.pixels,
    val horizontalExtentPixels: Pixels = 0.pixels,
    val verticalExtentPixels: Pixels = 0.pixels,
    val skew: Angle = 0.0.degrees
) {
    companion object {
        fun fromRawData(t2d: DoubleArray) =
            if (t2d.size != 17) LimelightDetection2d()
            else LimelightDetection2d(
                isValidData = true,
                hasValidTarget = t2d[LimelightIndices.T2d.targetValid] == 1.0,
                count = t2d[LimelightIndices.T2d.targetCount].toInt(),
                latency = t2d[LimelightIndices.T2d.targetLatency].milliseconds,
                captureLatency = t2d[LimelightIndices.T2d.captureLatency].milliseconds,
                horizontalAngularOffset = t2d[LimelightIndices.T2d.tx].degrees,
                verticalAngularOffset = t2d[LimelightIndices.T2d.ty].degrees,
                horizontalLinearOffset = t2d[LimelightIndices.T2d.txnc].toInt().pixels,
                verticalLinearOffset = t2d[LimelightIndices.T2d.tync].toInt().pixels,
                areaOccupancy = t2d[LimelightIndices.T2d.ta].percent,
                id = t2d[LimelightIndices.T2d.tid].toInt(),
                classifierClassIndex = t2d[LimelightIndices.T2d.targetClassIndexClassifier].toInt(),
                detectorClassIndex = t2d[LimelightIndices.T2d.targetClassIndexDetector].toInt(),
                longSidePixels = t2d[LimelightIndices.T2d.targetLongSidePixels].toInt().pixels,
                shortSidePixels = t2d[LimelightIndices.T2d.targetShortSidePixels].toInt().pixels,
                horizontalExtentPixels = t2d[LimelightIndices.T2d.targetHorizontalExtentPixels].toInt().pixels,
                verticalExtentPixels = t2d[LimelightIndices.T2d.targetVerticalExtentPixels].toInt().pixels,
                skew = t2d[LimelightIndices.T2d.targetSkewDegrees].degrees
            )
    }
}

/**
 * Stores a snapshot of 3-dimensional detection data
 */
data class LimelightDetection3d(
    val pose3d: Pose3d,
    val latency: Time,
    val tagCount: Int,
    val tagSpan: Double,
    val averageTagDistanceFromCamera: Double,
    val averageTagCoverage: Percentage
) {
    companion object {
        fun fromRawData(data: DoubleArray) = LimelightDetection3d(
            pose3d = Pose3d(
                Translation3d(
                    data[LimelightIndices.T3d.x],
                    data[LimelightIndices.T3d.y],
                    data[LimelightIndices.T3d.z]
                ),
                Rotation3d(
                    data[LimelightIndices.T3d.roll],
                    data[LimelightIndices.T3d.pitch],
                    data[LimelightIndices.T3d.yaw]

                )
            ),
            latency = data[LimelightIndices.T3d.latency].milliseconds,
            tagCount = data[LimelightIndices.T3d.tagCount].toInt(),
            tagSpan = data[LimelightIndices.T3d.tagSpan],
            averageTagDistanceFromCamera = data[LimelightIndices.T3d.averageTagDistanceFromCamera],
            averageTagCoverage = data[LimelightIndices.T3d.averageTagCoverage].percent
        )

        fun poseToRawData(pose3d: Pose3d) = arrayOf(pose3d.x, pose3d.y, pose3d.z, pose3d.rotation.x, pose3d.rotation.y, pose3d.rotation.z).toDoubleArray()
    }
}

/**
 * Helper class to create a pose estimation from [Limelight] detection.
 * Requires uploading a field file to the limelight dashboard
 */
data class LimelightPoseEstimate(
    val pose2d: Pose2d,
    val pose3d: Pose3d,
    val timestamp: Time,
    val latency: Time,
    val isValidEstimate: Boolean,
    val targetCount: Int,
    val tagSpan: Double,
    val averageTagDistance: Double,
    val averageTagArea: Double
)

/**
 * Stores a snapshot of a [Limelight]'s hardware metrics
 */
data class LimelightHardwareMetrics(
    val fps: Frequency,
    val cpuTemperature: Temperature,
    val temperature: Temperature,
    val ramUsage: Percentage
) {
    companion object {
        fun fromRawData(value: DoubleArray): LimelightHardwareMetrics =
            LimelightHardwareMetrics(
                fps = value[0].hertz,
                cpuTemperature = value[1].degreesCelsius,
                ramUsage = value[2].percent,
                temperature = value[3].degreesCelsius
            )
    }
}

enum class LimelightLedMode(val mode: Number) {
    Pipeline(0),
    Off(1),
    Blink(2),
    On(3)
}

enum class LimelightStreamMode(val mode: Number) {
    Standard(0),
    Main(1),
    Secondary(2)
}

data class LimelightCrop(
    val x0: Double,
    val y0: Double,
    val x1: Double,
    val y1: Double,
) {
    val array: Array<Number> = arrayOf(
        x0, x1, y0, y1
    )
}