package net.tecdroid.systems.ArmSystem

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import net.tecdroid.vision.limelight.systems.LimeLightChoice
import net.tecdroid.vision.limelight.systems.LimelightController

data class BranchChoice (
    var apriltagId: Int,
    var levelPose: PoseCommands,
    var sideChoice: LimeLightChoice
)

class ReefAppListener(llController: LimelightController): SubsystemBase() {
    private val reefAutoLevelSelector = ReefAutoLevelSelector(llController)

    // Network table
    private val table = NetworkTableInstance.getDefault().getTable("ReefAppData")

    // Branch choice object
    val branchChoice = BranchChoice(0, PoseCommands.BackL2, LimeLightChoice.Right)

    private fun shuffleboardData() {
        val tab = Shuffleboard.getTab("Driver Tab")
        tab.addString("Branch Choice", {
            branchChoice.apriltagId.toString() + " " + branchChoice.levelPose.toString() + " " + branchChoice.sideChoice.toString()
        })

    }

    init {
        shuffleboardData()
    }

    //fun getBetterLevel(aprilTagId: Int, branchSide: BranchSide): PoseCommands? = reefAutoLevelSelector.getBetterLevel(aprilTagId, branchSide)

    override fun periodic() {
        val apriltagId = table.getEntry("ApriltagId").getString("Apriltag id not found")
        val level = table.getEntry("Level").getString("Level not found")
        val side = table.getEntry("Side").getString("Side not found")
        val action = table.getEntry("ReefAction").getString("Action not found")

        // Select apriltag id
        val redIds = mapOf(
            "1" to 8, "2" to 7, "3" to 6,
            "4" to 11, "5" to 10, "6" to 9
        )

        val blueIds = mapOf(
            "1" to 17, "2" to 18, "3" to 19,
            "4" to 20, "5" to 21, "6" to 22
        )

        DriverStation.getAlliance().orElse(null)?.let { alliance ->
            val idMap = when (alliance) {
                Alliance.Red -> redIds
                Alliance.Blue -> blueIds
            }
            val numericId = idMap[apriltagId] ?: 0
            branchChoice.apriltagId = numericId
        }

        // Select level
        when (level) {
            "L2" -> branchChoice.levelPose = PoseCommands.BackL2
            "L3" -> branchChoice.levelPose = PoseCommands.BackL3
            "L4" -> branchChoice.levelPose = PoseCommands.BackL4
            else -> {}//println("Not registered pose")
        }

        // Select side
        when (side) {
            "right" -> branchChoice.sideChoice = LimeLightChoice.Right
            "left" -> branchChoice.sideChoice = LimeLightChoice.Left
            else -> {}//println("Not registered side")
        }

//        when (action) {
//            "fill" -> reefAutoLevelSelector.fillLevel(branchChoice) // Fill level in reef auto level selector
//            "empty" -> reefAutoLevelSelector.emptyLevel(branchChoice) // Fill level in reef auto level selector
//            else -> {}//println("Not registered action")
//        }


    }
}