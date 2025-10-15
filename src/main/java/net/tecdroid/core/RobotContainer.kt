package net.tecdroid.core

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import net.tecdroid.autonomous.PathPlannerAutonomous
import net.tecdroid.commands.DriveCommands
import net.tecdroid.constants.SwerveTunerConstants
import net.tecdroid.subsystems.drivetrain.Drive
import net.tecdroid.subsystems.drivetrain.GyroIO
import net.tecdroid.subsystems.drivetrain.GyroIOPigeon2
import net.tecdroid.subsystems.drivetrain.ModuleIO
import net.tecdroid.subsystems.drivetrain.ModuleIOSim
import net.tecdroid.subsystems.drivetrain.ModuleIOTalonFX
import net.tecdroid.constants.Constants
import net.tecdroid.constants.Constants.driverControllerId
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.systems.ArmSystem.ArmOrders
import net.tecdroid.systems.ArmSystem.ArmPoses
import net.tecdroid.systems.ArmSystem.ArmSystem
import net.tecdroid.systems.ArmSystem.BranchSide
import net.tecdroid.systems.ArmSystem.PoseCommands
import net.tecdroid.systems.ArmSystem.ReefAppListener
import net.tecdroid.systems.ArmSystem.ReefAutoLevelSelector
import net.tecdroid.systems.ArmSystem.Side
import net.tecdroid.systems.SwerveRotationLockSystem
import net.tecdroid.util.degrees
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
    private val llController: LimelightController
    //private val pathPlannerAutonomous: PathPlannerAutonomous
    private val swerveRotationLockSystem: SwerveRotationLockSystem
    private val reefAppListener: ReefAppListener

    private val xLimelightToAprilTagSetPoint = 0.315
    private val yLimelightToAprilTagSetPoint = 0.035
    private val visionStdDev = VecBuilder.fill(.5, .5, .2)
    private val pathPlannerAutonomous: PathPlannerAutonomous

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

        llController = LimelightController(
            drive,
            { chassisSpeeds -> drive.runVelocity(chassisSpeeds) },
            { drive.rotation.degrees }, drive.maxSwerveSpeeds.times(0.75))
        llController.shuffleboardData()
        arm.publishShuffleBoardData()
        arm.assignCommands()

        swerveRotationLockSystem = SwerveRotationLockSystem(drive, controller)
        reefAppListener = ReefAppListener(llController)
        pathPlannerAutonomous = PathPlannerAutonomous(drive, llController, arm)
    }


    fun autonomousInit() {
        drive.removeDefaultCommand()
    }

    fun disableInit() {
        llController.setThrottle(150)
        controller.a().and { DriverStation.isDisabled() }.onTrue(arm.setAllCoast())
        controller.b().and { DriverStation.isDisabled() }.onTrue(arm.setAllBrake())
    }

    fun teleopInit() {
        arm.setAllBrake()
        llController.setThrottle(0)

//        controller.a().onTrue(InstantCommand({ arm.climber.setRawAngle(140.0.degrees, 12.0.volts) }))
////        //controller.x().onTrue(InstantCommand({ arm.climber.setAngle(140.0.degrees) }))
////        //controller.y().onTrue(InstantCommand({ arm.climber.setAngle(100.0.degrees) }))
//        controller.b().onTrue(InstantCommand({ arm.climber.setRawAngle(33.5.degrees, 12.0.volts) }))
////
//        controller.rightBumper().onTrue(InstantCommand({arm.climber.setClimberRollersVoltage(8.0.volts)}))
//            .onFalse(InstantCommand({arm.climber.setClimberRollersVoltage(0.0.volts)}))
//
//        controller.x().onTrue(arm.setPoseCommand(ArmPoses.CoralFloorIntakeSafe.pose, ArmOrders.JEW.order))
//        controller.y().onTrue(arm.setPoseCommand(ArmPoses.BackL2.pose, ArmOrders.JEW.order))

        controller.povRight().onTrue(InstantCommand({ arm.setPoseCommand(PoseCommands.CoralStation.pose, ArmOrders.JEW.order) }))



        // Reset gyro to 0° when Start button is pressed
        controller.start().onTrue(
                Commands.runOnce(
                    Runnable {
                        drive.pose = Pose2d(drive.pose.translation, Rotation2d())
                    },
                    drive
                )
            )

        // Default command, normal field-relative drive
        drive.defaultCommand = DriveCommands.joystickDrive(
            drive,
            DoubleSupplier { -controller.getLeftY() * 0.8 },
            DoubleSupplier { -controller.getLeftX() * 0.8 },
            DoubleSupplier { controller.getRightX() * 0.6 })

        controller.rightTrigger().whileTrue(llController
            .alignRobotAllAxis({ llController.getLimelight(BranchSide.Right) }) { llController.getRightLLSetpoints(arm.currentPose) })
        controller.leftTrigger().whileTrue(llController
            .alignRobotAllAxis({ llController.getLimelight(BranchSide.Left) }) { llController.getLeftLLSetpoints(arm.currentPose) })

        // Auto Level Selector

        controller.back().onTrue(Commands.runOnce({ autoLevelSelectorMode = !autoLevelSelectorMode }))

//        controller.rightTrigger().and { limeLightIsAtSetPoint(if (llController.isFront) Front else Right) && autoLevelSelectorMode }
//            .onTrue(Commands.runOnce({ betterLevelSequence(llController.getLimelight(BranchSide.Right), BranchSide.Right) }))
//
//        controller.leftTrigger().and { limeLightIsAtSetPoint(if (llController.isFront) Front else Left) && autoLevelSelectorMode }
//            .onTrue(Commands.runOnce({ betterLevelSequence(if (llController.isFront) Front else Left) }))
//
//        controller.rightTrigger().whileTrue(llController
//            .alignRobotAllAxis({ reefAppListener.branchChoice.sideChoice },
//                llController.getRightLLSetpoints(if (llController.isFront) Front else Right)))
//        Trigger { limeLightIsAtSetPoint(reefAppListener.branchChoice.sideChoice) }.onTrue(arm.scoringSequence({ reefAppListener.branchChoice.levelPose }))

        //States.IntakeState.setDefaultCommand(swerveRotationLockSystem.lockRotationCMD(LockPositions.CoralStation))
        //States.IntakeState.setEndCommand(Commands.runOnce({swerve.currentCommand.cancel()}))
    }

//    private fun betterLevelSequence(llChoice: LimeLightChoice, branchSide: BranchSide) {
//        reefAppListener.getBetterLevel(
//            llController.getTargetId(llChoice),
//            branchSide
//        )?.let {
//            arm.scoringSequence(it).schedule()
//        } ?: Commands.none()
//    }

    private fun advantageScopeLogs() {
        robotPosePublisher.set(drive.pose)
    }

    fun limeLightIsAtSetPoint(limeLightChoice: LimeLightChoice): Boolean {
        return when (limeLightChoice) {
            Right -> llController.isAtSetPoint(Right, llController.getRightLLSetpoints(arm.currentPose))
            Left -> llController.isAtSetPoint(Left, llController.getLeftLLSetpoints(arm.currentPose))
            Front -> llController.isAtSetPoint(Front, llController.getRightLLSetpoints(arm.currentPose)) ||
                    llController.isAtSetPoint(Front, llController.getLeftLLSetpoints(arm.currentPose))
        }
    }

    fun limeLightIsAtSetPoint(tolerance: Distance): Boolean {
         return llController.isAtSetPoint(Front, llController.getRightLLSetpoints(arm.currentPose), tolerance) ||
                llController.isAtSetPoint(Front, llController.getLeftLLSetpoints(arm.currentPose), tolerance) ||
                llController.isAtSetPoint(Right, llController.getRightLLSetpoints(arm.currentPose), tolerance) ||
                llController.isAtSetPoint(Left, llController.getLeftLLSetpoints(arm.currentPose), tolerance)
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
        get() = pathPlannerAutonomous.selectedAutonomousRoutine

}
