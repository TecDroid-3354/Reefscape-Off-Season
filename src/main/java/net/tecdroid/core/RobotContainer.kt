package net.tecdroid.core

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import net.tecdroid.commands.DriveCommands
import net.tecdroid.constants.SwerveTunerConstants
import net.tecdroid.subsystems.drivetrain.Drive
import net.tecdroid.subsystems.drivetrain.GyroIO
import net.tecdroid.subsystems.drivetrain.GyroIOPigeon2
import net.tecdroid.subsystems.drivetrain.ModuleIO
import net.tecdroid.subsystems.drivetrain.ModuleIOSim
import net.tecdroid.subsystems.drivetrain.ModuleIOTalonFX
import net.tecdroid.autonomous.PathPlannerAutonomous
import net.tecdroid.constants.Constants
import net.tecdroid.constants.Constants.driverControllerId
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.systems.ArmSystem.ArmSystem
import net.tecdroid.systems.ArmSystem.PoseCommands
import net.tecdroid.systems.ArmSystem.ReefAppListener
import net.tecdroid.systems.SwerveRotationLockSystem
import net.tecdroid.util.degrees
import net.tecdroid.util.seconds
import net.tecdroid.util.stateMachine.StateMachine
import net.tecdroid.util.stateMachine.States
import net.tecdroid.util.volts
import net.tecdroid.vision.limelight.systems.LimeLightChoice
import net.tecdroid.vision.limelight.systems.LimeLightChoice.*
import net.tecdroid.vision.limelight.systems.LimelightController
import java.util.function.DoubleSupplier


class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private var drive: Drive
    private val stateMachine = StateMachine(States.CoralState)
    private val arm = ArmSystem(stateMachine, ::limeLightIsAtSetPoint, controller)
    private val limelightController: LimelightController
    //private val pathPlannerAutonomous: PathPlannerAutonomous
    private val swerveRotationLockSystem: SwerveRotationLockSystem
    private val reefAppListener = ReefAppListener()

    private val xLimelightToAprilTagSetPoint = 0.315
    private val yLimelightToAprilTagSetPoint = 0.035
    private val visionStdDev = VecBuilder.fill(.5, .5, .2)

    private var autoLevelSelectorMode = true

    // Advantage Scope log publisher
    private val robotPosePublisher: StructPublisher<Pose2d> = NetworkTableInstance.getDefault()
        .getStructTopic("RobotPose", Pose2d.struct).publish()

    init {
        when (Constants.currentMode) {
            Constants.Mode.REAL ->         // Real robot, instantiate hardware IO implementations
                drive =
                    Drive(
                        GyroIOPigeon2(),
                        ModuleIOTalonFX(SwerveTunerConstants.FrontLeft),
                        ModuleIOTalonFX(SwerveTunerConstants.FrontRight),
                        ModuleIOTalonFX(SwerveTunerConstants.BackLeft),
                        ModuleIOTalonFX(SwerveTunerConstants.BackRight)
                    )

            Constants.Mode.SIM ->         // Sim robot, instantiate physics sim IO implementations
                drive =
                    Drive(
                        object : GyroIO {},
                        ModuleIOSim(SwerveTunerConstants.FrontLeft),
                        ModuleIOSim(SwerveTunerConstants.FrontRight),
                        ModuleIOSim(SwerveTunerConstants.BackLeft),
                        ModuleIOSim(SwerveTunerConstants.BackRight)
                    )

            else ->         // Replayed robot, disable IO implementations
                drive =
                    Drive(
                        object : GyroIO {},
                        object : ModuleIO {},
                        object : ModuleIO {},
                        object : ModuleIO {},
                        object : ModuleIO {})
        }

        limelightController = LimelightController(
            drive,
            { chassisSpeeds -> drive.runVelocity(chassisSpeeds) },
            { drive.rotation.degrees }, drive.maxSwerveSpeeds.times(0.75))
        limelightController.shuffleboardData()
        arm.publishShuffleBoardData()
        arm.assignCommands()

        swerveRotationLockSystem = SwerveRotationLockSystem(drive, controller)
        //pathPlannerAutonomous = PathPlannerAutonomous(drive, limelightController, arm)
    }


    fun autonomousInit() {
        //swerve.removeDefaultCommand()
    }

    fun disableInit() {
        limelightController.setThrottle(150)
        controller.a().and { DriverStation.isDisabled() }.onTrue(arm.setAllCoast())
        controller.b().and { DriverStation.isDisabled() }.onTrue(arm.setAllBrake())
    }

    fun teleopInit() {
        arm.setAllBrake()
        limelightController.setThrottle(0)

//        controller.rightTrigger().onTrue(InstantCommand({arm.climber.setVoltage(2.0.volts)}))
//            .onFalse(InstantCommand({arm.climber.setVoltage(0.0.volts)}))
//
//        controller.rightTrigger().onTrue(InstantCommand({arm.climber.setVoltage(-(2.0).volts)}))
//            .onFalse(InstantCommand({arm.climber.setVoltage(0.0.volts)}))

        // Reset gyro to 0° when Start button is pressed
        controller.start().onTrue(
                Commands.runOnce(
                    Runnable {
                        drive.pose = Pose2d(drive.pose.translation, Rotation2d())
                    },
                    drive
                )
            )

        //limelightController.setFilterIds(arrayOf(21, 20, 19, 18, 17, 22, 10, 11, 6, 7, 8, 9))

        // Default command, normal field-relative drive
        drive.defaultCommand = DriveCommands.joystickDrive(
            drive,
            DoubleSupplier { -controller.getLeftY() * 0.8 },
            DoubleSupplier { -controller.getLeftX() * 0.8 },
            DoubleSupplier { controller.getRightX() * 0.5 })

        controller.rightTrigger().whileTrue(limelightController
            .alignRobotAllAxis(Right, limelightController.getRightLLSetpoints(if (limelightController.isFront) Front else Right)))
        controller.leftTrigger().whileTrue(limelightController
            .alignRobotAllAxis(Left, limelightController.getRightLLSetpoints(if (limelightController.isFront) Front else Left)))

        // Auto Level Selector

        controller.back().onTrue(Commands.runOnce({ autoLevelSelectorMode = !autoLevelSelectorMode }))

        controller.rightTrigger().and { limeLightIsAtSetPoint(if (limelightController.isFront) Front else Right) && autoLevelSelectorMode }
            .onTrue(Commands.runOnce({ betterLevelSequence(if (limelightController.isFront) Front else Right) }))

        controller.leftTrigger().and { limeLightIsAtSetPoint(if (limelightController.isFront) Front else Left) && autoLevelSelectorMode }
            .onTrue(Commands.runOnce({ betterLevelSequence(if (limelightController.isFront) Front else Left) }))

        controller.rightTrigger().whileTrue(limelightController
            .alignRobotAllAxis({ reefAppListener.branchChoice.sideChoice },
                limelightController.getRightLLSetpoints(if (limelightController.isFront) Front else Right)))
        Trigger { limeLightIsAtSetPoint(reefAppListener.branchChoice.sideChoice) }.onTrue(arm.scoringSequence({ reefAppListener.branchChoice.levelPose }))

        //States.IntakeState.setDefaultCommand(swerveRotationLockSystem.lockRotationCMD(LockPositions.CoralStation))
        //States.IntakeState.setEndCommand(Commands.runOnce({swerve.currentCommand.cancel()}))
    }

    private fun betterLevelSequence(choice: LimeLightChoice) {
        reefAppListener.getBetterLevel(
            limelightController.getTargetId(choice),
            choice
        )?.let {
            arm.scoringSequence(it).schedule()
        } ?: Commands.none()
    }

    private fun advantageScopeLogs() {
        robotPosePublisher.set(drive.pose)
    }

    fun limeLightIsAtSetPoint(limeLightChoice: LimeLightChoice): Boolean {
        return when (limeLightChoice) {
            Right -> limelightController.isAtSetPoint(Right, limelightController.getRightLLSetpoints(Right))
            Left -> limelightController.isAtSetPoint(Left, limelightController.getLeftLLSetpoints(Left))
            Front -> limelightController.isAtSetPoint(Front, limelightController.getRightLLSetpoints(Right)) ||
                    limelightController.isAtSetPoint(Front, limelightController.getLeftLLSetpoints(Left))
        }
    }

    fun limeLightIsAtSetPoint(tolerance: Distance): Boolean {
         return limelightController.isAtSetPoint(Front, limelightController.getRightLLSetpoints(Front), tolerance) ||
                limelightController.isAtSetPoint(Front, limelightController.getLeftLLSetpoints(Front), tolerance)
                limelightController.isAtSetPoint(Right, limelightController.getRightLLSetpoints(Right), tolerance) ||
                limelightController.isAtSetPoint(Left, limelightController.getLeftLLSetpoints(Left), tolerance)
    }


    fun robotPeriodic() {
        advantageScopeLogs()

//        try {
//            if (limelightController.hasTarget(LimeLightChoice.Left)) {
//                drive.addVisionMeasurement(
//                    limelightController.getRobotPoseEstimate(LimeLightChoice.Left).pose,
//                    limelightController.getRobotPoseEstimate(LimeLightChoice.Left).timestampSeconds,
//                    visionStdDev
//                )
//            }
//        } catch (e: Exception) {
//            println("left limelight pose update error")
//        }
//
//        try {
//            if (limelightController.hasTarget(LimeLightChoice.Right)) {
//                drive.addVisionMeasurement(
//                    limelightController.getRobotPoseEstimate(LimeLightChoice.Right).pose,
//                    limelightController.getRobotPoseEstimate(LimeLightChoice.Right).timestampSeconds,
//                    visionStdDev
//                )
//            }
//        } catch (e: Exception) {
//            println("Right limelight update error")
//        }

    }

    val autonomousCommand: Command
        get() = Commands.none()//pathPlannerAutonomous.selectedAutonomousRoutine

}
