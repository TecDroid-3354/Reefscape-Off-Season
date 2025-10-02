package net.tecdroid.autonomous

import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import net.tecdroid.subsystems.drivetrain.Drive
import net.tecdroid.systems.ArmSystem.ArmOrders
import net.tecdroid.systems.ArmSystem.ArmPoses
import net.tecdroid.systems.ArmSystem.ArmSystem
import net.tecdroid.util.seconds
import net.tecdroid.vision.limelight.systems.LimeLightChoice
import net.tecdroid.vision.limelight.systems.LimelightController
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import java.io.IOException

class PathPlannerAutonomous(val drive: Drive, private val limelightController: LimelightController, private val armSystem: ArmSystem) {
    private val autoChooser = LoggedDashboardChooser<Command>("Auto Choices", drive.autoChooser)

    private val robotConfig: RobotConfig = try {
        RobotConfig.fromGUISettings()
    } catch (e: Exception) {
        DriverStation.reportError(
            "Could not initialize Robot Configuration for a Path Planner Autonomous Config",
            e.stackTrace
        )
        throw IOException(e)
    }

    private val driveController = PPHolonomicDriveController(
        PIDConstants(6.0, 0.0, 0.0),
        PIDConstants(18.0, 0.0, 0.0)
    )

    private fun registerNamedCommand(name: String, command: Command) {
        NamedCommands.registerCommand(name, command)
    }

    private fun namedCommandsInit() {
        // Stop motors
        registerNamedCommand("StopMotors",
            drive.stopCommand())
        // Arm
        registerNamedCommand("ArmCoralStationPose",
            armSystem.setPoseAutoCommand(ArmPoses.CoralStation.pose, ArmOrders.EJW.order))

        registerNamedCommand("ArmL4Pose",
            armSystem.setPoseAutoCommand(ArmPoses.L4.pose, ArmOrders.JEW.order))

        registerNamedCommand("ArmL3PoseWJE",
        armSystem.setPoseAutoCommand(ArmPoses.L3.pose, ArmOrders.WEJ.order))

        registerNamedCommand("ArmL3PoseEWJ",
            armSystem.setPoseAutoCommand(ArmPoses.L3.pose, ArmOrders.EWJ.order))

        registerNamedCommand("ArmL2PoseWJE",
            armSystem.setPoseAutoCommand(ArmPoses.L2.pose, ArmOrders.WJE.order))

        registerNamedCommand("ArmL2PoseJEW",
            armSystem.setPoseAutoCommand(ArmPoses.L2.pose, ArmOrders.JEW.order))

        // Intake
        registerNamedCommand("EnableIntakeUntilHasCoral",
            Commands.sequence(
                armSystem.enableCoralIntake(),
                Commands.waitUntil { armSystem.intake.hasCoral() },
                armSystem.disableCoralIntake()
            ))

        registerNamedCommand("EnableIntake",
            armSystem.enableCoralIntake())

        // Score commands

        registerNamedCommand("AlignAndScoreRightBranch",
            Commands.sequence(
                ParallelCommandGroup(
                    limelightController.alignRobotAllAxis(LimeLightChoice.Right, 0.215, 0.035)
                        .until { limelightController.isAtSetPoint(LimeLightChoice.Right, 0.215, 0.035) },
                    armSystem.setPoseAutoCommand(ArmPoses.L4.pose, ArmOrders.JEW.order),
                ).withTimeout(2.5),
                drive.stopCommand(),

                armSystem.enableCoralIntake(),
                Commands.waitUntil { !armSystem.intake.hasCoral() },
                Commands.waitTime(0.35.seconds),
                armSystem.disableCoralIntake())
            )

        registerNamedCommand("AlignAndScoreLeftBranch",
            Commands.sequence(
                ParallelCommandGroup(
                    limelightController.alignRobotAllAxis(LimeLightChoice.Left, 0.215, -0.035)
                        .until { limelightController.isAtSetPoint(LimeLightChoice.Left, 0.215, -0.035) },
                    armSystem.setPoseAutoCommand(ArmPoses.L4.pose, ArmOrders.JEW.order),
                ).withTimeout(2.5),

                drive.stopCommand(),

                armSystem.enableCoralIntake(),
                Commands.waitUntil { !armSystem.intake.hasCoral() },
                Commands.waitTime(0.35.seconds),
                armSystem.disableCoralIntake())
            )

        registerNamedCommand("AlignAndScoreRightBranchIdFilter",
            Commands.sequence(
                Commands.runOnce({limelightController.setFilterIds(arrayOf(20, 19, 11, 6));}),
                ParallelCommandGroup(
                    limelightController.alignRobotAllAxis(LimeLightChoice.Right, 0.215, -0.035)
                        .until { limelightController.isAtSetPoint(LimeLightChoice.Right, 0.215, -0.035) },
                    armSystem.setPoseAutoCommand(ArmPoses.L4.pose, ArmOrders.JEW.order),
                ).withTimeout(2.5),

                drive.stopCommand(),

                Commands.runOnce({limelightController.setFilterIds(arrayOf(21, 20, 19, 18, 17, 22, 10, 11, 6, 7, 8, 9));}),

                armSystem.enableCoralIntake(),
                Commands.waitUntil { !armSystem.intake.hasCoral() },
                Commands.waitTime(0.35.seconds),
                armSystem.disableCoralIntake())
        )
    }

    private fun autoChooserOptions() {
        val tab = Shuffleboard.getTab("Driver Tab")
        autoChooser.addDefaultOption("None", Commands.none())

        autoChooser.addOption("Straight Forward", resetPoseAndGetPathFollowingCommand("Straightforward"))

        // Complete autos
        autoChooser.addOption("RightAuto", PathPlannerAuto("Right Auto"))
        autoChooser.addOption("LeftAuto", PathPlannerAuto("Left Auto"))
        //autoChooser.addOption("CenterAuto", PathPlannerAuto("Center Auto"))
        autoChooser.addOption("CenterAuto",
            Commands.sequence(
                Commands.waitTime(1.5.seconds),
                Commands.runOnce({limelightController.setFilterIds(arrayOf(10, 21));}),
                ParallelCommandGroup(
                    limelightController.alignRobotAllAxis(LimeLightChoice.Right, 0.215, 0.035)
                        .until { limelightController.isAtSetPoint(LimeLightChoice.Right, 0.215, 0.035) },
                    armSystem.setPoseAutoCommand(ArmPoses.L4.pose, ArmOrders.JEW.order),
                ).withTimeout(2.5),
                drive.stopCommand(),

                Commands.runOnce({limelightController.setFilterIds(arrayOf(21, 20, 19, 18, 17, 22, 10, 11, 6, 7, 8, 9));}),

                armSystem.enableCoralIntake(),
                Commands.waitUntil { !armSystem.intake.hasCoral() },
                Commands.waitTime(0.35.seconds),
                armSystem.disableCoralIntake(),
                armSystem.setPoseAutoCommand(ArmPoses.L2.pose, ArmOrders.JEW.order)))

        tab.add("Autonomous Chooser", autoChooser.sendableChooser)
        SmartDashboard.putData("Autonomous Chooser", autoChooser.sendableChooser)
    }

    init {
        var alliance = DriverStation.getAlliance()

        // Instead I used AutoBuilder inside Drive. Should see why configuring it here gives me an error,
        // I suspect is due to Java - Kotlin interaction failing.
//        AutoBuilder.configure(
//            drive.pose,
//            drive.pose,
//            drive.chassisSpeeds,
//            {speeds: ChassisSpeeds -> drive.runVelocity(speeds)},
//            driveController,
//            robotConfig,
//            { if (alliance.isPresent) { alliance.get() == Alliance.Red } else false },
//            drive
//        )

        namedCommandsInit()
        autoChooserOptions()
    }

    val selectedAutonomousRoutine: Command
        get() = if (autoChooser.get() != null) autoChooser.get() else Commands.none()

    fun getPath(name: String): PathPlannerPath = try {
        PathPlannerPath.fromPathFile(name)
    } catch (e: Exception) {
        DriverStation.reportError("Path Planner Autonomous Error", false)
        throw e
    }


    fun getPathFollowingCommand(name: String): Command = drive.followTrajectory(getPath(name))
    fun getPathFollowingCommand(path: PathPlannerPath): Command = drive.followTrajectory(path)

    fun resetPoseAndGetPathFollowingCommand(name: String) : Command {
        val path = getPath(name)
        return resetPoseAndGetPathFollowingCommand(path)
    }

    private fun resetPoseAndGetPathFollowingCommand(path: PathPlannerPath) : Command {
        return Commands.runOnce({
            drive.pose = path.pathPoses.first()
            SmartDashboard.putBoolean("SSS", true)
        }).andThen(getPathFollowingCommand(path))
    }
}
