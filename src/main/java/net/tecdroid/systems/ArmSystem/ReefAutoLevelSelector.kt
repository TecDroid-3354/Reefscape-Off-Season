package net.tecdroid.systems.ArmSystem

import net.tecdroid.util.NumericId
import net.tecdroid.vision.limelight.systems.LimeLightChoice

data class Level(val poseCommand: PoseCommands, var occupied: Boolean = false)

data class Branch(val L2: Level = Level(PoseCommands.L2), val L3: Level = Level(PoseCommands.L3),
                  val L4: Level = Level(PoseCommands.L4))

data class Side(val leftBranch: Branch = Branch(), val rightBranch: Branch = Branch())

data class Reef(val side1: Side = Side(), val side2: Side = Side(),
                val side3: Side = Side(), val side4: Side = Side(),
                val side5: Side = Side(), val side6: Side = Side())

class ReefAutoLevelSelector {
    val reef = Reef()

    val sideMap = mapOf(
        8 to reef.side1, 17 to reef.side1,
        7 to reef.side2, 18 to reef.side2,
        6 to reef.side3, 19 to reef.side3,
        11 to reef.side4, 20 to reef.side4,
        10 to reef.side5, 21 to reef.side5,
        9 to reef.side6, 22 to reef.side6
    )

    fun getBetterLevel(aprilTagId: Int, limeLightChoice: LimeLightChoice): PoseCommands? {
        return sideMap[aprilTagId]?.let { side ->
            val choice = when (limeLightChoice) {
                LimeLightChoice.Left -> side.leftBranch
                LimeLightChoice.Right -> side.rightBranch
            }

            listOf(choice.L4, choice.L3, choice.L2)
                .firstOrNull { !it.occupied }
                ?.poseCommand
        }
    }

    fun fillLevel(branchChoice: BranchChoice) {
        sideMap[branchChoice.apriltagId]?.let { side ->
            val choice = when (branchChoice.sideChoice) {
                LimeLightChoice.Left -> side.leftBranch
                LimeLightChoice.Right -> side.rightBranch
            }

            when (branchChoice.levelPose) {
                PoseCommands.L4 -> choice.L4.occupied = true
                PoseCommands.L3 -> choice.L3.occupied = true
                PoseCommands.L2 -> choice.L2.occupied = true
                PoseCommands.CoralStation -> TODO("No se puede seleccionar coral station como nivel")
                PoseCommands.Processor -> TODO("No se puede seleccionar processor como nivel")
            }
        }
    }

    fun emptyLevel(branchChoice: BranchChoice) {
        sideMap[branchChoice.apriltagId]?.let { side ->
            val choice = when (branchChoice.sideChoice) {
                LimeLightChoice.Left -> side.leftBranch
                LimeLightChoice.Right -> side.rightBranch
            }

            when (branchChoice.levelPose) {
                PoseCommands.L4 -> choice.L4.occupied = false
                PoseCommands.L3 -> choice.L3.occupied = false
                PoseCommands.L2 -> choice.L2.occupied = false
                PoseCommands.CoralStation -> TODO("No se puede seleccionar coral station como nivel")
                PoseCommands.Processor -> TODO("No se puede seleccionar processor como nivel")

            }
        }
    }
}