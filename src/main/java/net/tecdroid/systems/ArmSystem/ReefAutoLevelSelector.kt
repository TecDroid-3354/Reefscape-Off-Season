package net.tecdroid.systems.ArmSystem

import edu.wpi.first.wpilibj.Alert
import net.tecdroid.vision.limelight.systems.LimeLightChoice
import net.tecdroid.vision.limelight.systems.LimelightController

data class Level(val poseCommand: PoseCommands, var occupied: Boolean = false)

data class Branch(val BackL2: Level = Level(PoseCommands.BackL2), val BackL3: Level = Level(PoseCommands.BackL3),
                  val BackL4: Level = Level(PoseCommands.BackL4))

data class Side(val leftBranch: Branch = Branch(), val rightBranch: Branch = Branch())
enum class BranchSide { Right, Left }

data class Reef(val side1: Side = Side(), val side2: Side = Side(),
                val side3: Side = Side(), val side4: Side = Side(),
                val side5: Side = Side(), val side6: Side = Side())

class ReefAutoLevelSelector(private val llController: LimelightController) {
    val reef = Reef()

    val sideMap = mapOf(
        8 to reef.side1, 17 to reef.side1,
        7 to reef.side2, 18 to reef.side2,
        6 to reef.side3, 19 to reef.side3,
        11 to reef.side4, 20 to reef.side4,
        10 to reef.side5, 21 to reef.side5,
        9 to reef.side6, 22 to reef.side6
    )

//    fun getBetterLevel(aprilTagId: Int, branchSide: BranchSide): PoseCommands? {
//        return sideMap[aprilTagId]?.let { side ->
//            val choice = llController.getLimelight(branchSide)}
//
//            listOf(choice.BackL4, choice.BackL3, choice.BackL2)
//                .firstOrNull { !it.occupied }
//                ?.poseCommand
//        }
//    }
//
//    fun fillLevel(branchChoice: BranchChoice) {
//        sideMap[branchChoice.apriltagId]?.let { side ->
//            val choice = when (branchChoice.sideChoice) {
//                LimeLightChoice.Left -> side.leftBranch
//                LimeLightChoice.Right -> side.rightBranch
//                LimeLightChoice.Front -> side.rightBranch // TODO() = Front logic to choose branch
//
//            }
//
//            when (branchChoice.levelPose) {
//                PoseCommands.BackL4 -> choice.BackL4.occupied = true
//                PoseCommands.BackL3 -> choice.BackL3.occupied = true
//                PoseCommands.BackL2 -> choice.BackL2.occupied = true
//                PoseCommands.CoralStation -> Alert("CoralStation is not a Reef level", Alert.AlertType.kError)
//                PoseCommands.Processor -> Alert("Processor is not a Reef level", Alert.AlertType.kError)
//                PoseCommands.Passive -> Alert("Passive is not a Reef level", Alert.AlertType.kError)
//            }
//        }
//    }
//
//    fun emptyLevel(branchChoice: BranchChoice) {
//        sideMap[branchChoice.apriltagId]?.let { side ->
//            val choice = when (branchChoice.sideChoice) {
//                LimeLightChoice.Left -> side.leftBranch
//                LimeLightChoice.Right -> side.rightBranch
//                LimeLightChoice.Front -> side.rightBranch // TODO () = Front logic to choose branch
//            }
//
//            when (branchChoice.levelPose) {
//                PoseCommands.BackL4 -> choice.BackL4.occupied = false
//                PoseCommands.BackL3 -> choice.BackL3.occupied = false
//                PoseCommands.BackL2 -> choice.BackL2.occupied = false
//                PoseCommands.CoralStation -> Alert("CoralStation is not a Reef level", Alert.AlertType.kError)
//                PoseCommands.Processor -> Alert("Processor is not a Reef level", Alert.AlertType.kError)
//                PoseCommands.Passive -> Alert("Passive is not a Reef level", Alert.AlertType.kError)
//            }
//        }
//    }
}