package net.tecdroid.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
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

class PathPlannerAutonomous(val drive: Drive, private val llController: LimelightController, private val armSystem: ArmSystem) {
    //private val autoChooser = LoggedDashboardChooser<Command>("Auto Choices", drive.autoChooser)
    private val autoChooser = SendableChooser<Command>()

    private val robotConfig: RobotConfig = try {
        RobotConfig.fromGUISettings()
    } catch (e: Exception) {
        DriverStation.reportError(
            "Could not initialize Robot Configuration for a Path Planner Autonomous Config",
            e.stackTrace
        )
        throw IOException(e)
    }

    private fun registerNamedCommand(name: String, command: Command) {
        NamedCommands.registerCommand(name, command)
    }

    private fun namedCommandsInit() {
        // Stop motors
        registerNamedCommand("StopMotors",
            drive.stopCommand())

        // Arm
        registerNamedCommand("ArmCoralStationPoseEJW",
            armSystem.setPoseAutoCommand(ArmPoses.CoralStation, ArmOrders.EJW.order))

        registerNamedCommand("ArmBackL4Pose",
            armSystem.setPoseAutoCommand(ArmPoses.BackL4, ArmOrders.JEW.order))

        registerNamedCommand("ArmBackL3PoseWJE",
        armSystem.setPoseAutoCommand(ArmPoses.BackL3, ArmOrders.WEJ.order))

        registerNamedCommand("ArmBackL3PoseEWJ",
            armSystem.setPoseAutoCommand(ArmPoses.BackL3, ArmOrders.EWJ.order))

        registerNamedCommand("ArmBackL2PoseWJE",
            armSystem.setPoseAutoCommand(ArmPoses.BackL2, ArmOrders.WJE.order))

        registerNamedCommand("ArmBackL2PoseJEW",
            armSystem.setPoseAutoCommand(ArmPoses.BackL2, ArmOrders.JEW.order))

        registerNamedCommand("FloorIntakePos",
            armSystem.setPoseAutoCommand(ArmPoses.CoralFloorIntake, ArmOrders.EJW.order))

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
                    llController.alignRobotAllAxis({ LimeLightChoice.Right }, { llController.getRightLLSetpoints(ArmPoses.BackL4) })
                        .until { llController.isAtSetPoint(LimeLightChoice.Right, llController.getRightLLSetpoints(ArmPoses.BackL4)) },
                    armSystem.setPoseAutoCommand(ArmPoses.BackL4, ArmOrders.JEW.order),
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
                    llController.alignRobotAllAxis(
                        { LimeLightChoice.Left },
                        { llController.getLeftLLSetpoints(ArmPoses.BackL4) })
                        .until { llController.isAtSetPoint(LimeLightChoice.Left, llController.getLeftLLSetpoints(ArmPoses.BackL4)) },
                    armSystem.setPoseAutoCommand(ArmPoses.BackL4, ArmOrders.JEW.order),
                ).withTimeout(2.5),

                drive.stopCommand(),

                armSystem.enableCoralIntake(),
                Commands.waitUntil { !armSystem.intake.hasCoral() },
                Commands.waitTime(0.35.seconds),
                armSystem.disableCoralIntake())
            )
    }

    private fun autoChooserOptions() {
        val tab = Shuffleboard.getTab("Driver Tab")
        autoChooser.setDefaultOption("None", Commands.none())

        autoChooser.addOption("Straight Forward", resetPoseAndGetPathFollowingCommand("Straightforward"))
        autoChooser.addOption("C1-CD-bargeToReef", resetPoseAndGetPathFollowingCommand("C1-CD-bargeToReef"))

        // Complete autos
        //autoChooser.addOption("RightAuto", PathPlannerAuto("Right Auto"))
        //autoChooser.addOption("LeftAuto", PathPlannerAuto("Left Auto"))
        //autoChooser.addOption("CenterAuto", PathPlannerAuto("Center Auto"))
        autoChooser.addOption("CenterAuto",
            Commands.sequence(
                Commands.waitTime(1.5.seconds),
                Commands.runOnce({llController.setFilterIds(arrayOf(10, 21));}),
                ParallelCommandGroup(
                    llController.alignRobotAllAxis(
                        { LimeLightChoice.Right },
                        { llController.getRightLLSetpoints(ArmPoses.BackL4) })
                        .until { llController.isAtSetPoint(LimeLightChoice.Right, llController.getRightLLSetpoints(ArmPoses.BackL4)) },
                    armSystem.setPoseAutoCommand(ArmPoses.BackL4, ArmOrders.JEW.order),
                ).withTimeout(2.5),
                drive.stopCommand(),

                Commands.runOnce({llController.setFilterIds(arrayOf(21, 20, 19, 18, 17, 22, 10, 11, 6, 7, 8, 9));}),

                armSystem.enableCoralIntake(),
                Commands.waitUntil { !armSystem.intake.hasCoral() },
                Commands.waitTime(0.35.seconds),
                armSystem.disableCoralIntake(),
                armSystem.setPoseAutoCommand(ArmPoses.BackL2, ArmOrders.JEW.order)))

        tab.add("Autonomous Chooser", autoChooser)
        SmartDashboard.putData(autoChooser)
    }

    init {
        var alliance = DriverStation.getAlliance()

        // Instead I used AutoBuilder inside Drive. Should see why configuring it here gives me an error,
        // I suspect is due to Java - Kotlin interaction failing.
        AutoBuilder.configure(
            drive::getPose,
            drive::setPose,
            drive::getChassisSpeeds,
            {speeds: ChassisSpeeds -> drive.runVelocity(speeds)},
            PPHolonomicDriveController(
                PIDConstants(0.4, 0.0, 0.01), PIDConstants(0.2, 0.0, 0.1)),
            robotConfig,
            { if (alliance.isPresent) { alliance.get() == Alliance.Red } else false },
            drive
        )

        namedCommandsInit()
        autoChooserOptions()
    }

    val selectedAutonomousRoutine: Command
        get() = autoChooser.selected ?: Commands.print("No auto found :(")

    fun getPath(name: String): PathPlannerPath = try {
        PathPlannerPath.fromPathFile(name)
    } catch (e: Exception) {
        DriverStation.reportError("Path Planner Autonomous Error", false)
        throw e
    }


    fun getPathFollowingCommand(name: String): Command = drive.followTrajectory(getPath(name))
    //fun getPathFollowingCommand(path: PathPlannerPath): Command = drive.followTrajectory(path)
    fun getPathFollowingCommand(path: PathPlannerPath): Command = AutoBuilder.followPath(path)

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
