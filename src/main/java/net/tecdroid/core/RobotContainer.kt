package net.tecdroid.core

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import net.tecdroid.autonomous.PathPlannerAutonomous
import net.tecdroid.constants.GenericConstants.driverControllerId
import net.tecdroid.input.CompliantXboxController
import net.tecdroid.subsystems.drivetrain.SwerveDrive
import net.tecdroid.subsystems.drivetrain.swerveDriveConfiguration
import net.tecdroid.systems.ArmSystem.ArmSystem
import net.tecdroid.systems.ArmSystem.ReefAppListener
import net.tecdroid.systems.SwerveRotationLockSystem
import net.tecdroid.util.degrees
import net.tecdroid.util.stateMachine.StateMachine
import net.tecdroid.util.stateMachine.States
import net.tecdroid.vision.limelight.systems.LimeLightChoice
import net.tecdroid.vision.limelight.systems.LimelightController


class RobotContainer {
    private val controller = CompliantXboxController(driverControllerId)
    private val swerve = SwerveDrive(swerveDriveConfiguration)
    private val stateMachine = StateMachine(States.CoralState)
    private val arm = ArmSystem(stateMachine, ::limeLightIsAtSetPoint, controller)
    private val limelightController = LimelightController(
        swerve,
        { chassisSpeeds -> swerve.driveRobotOriented(chassisSpeeds) },
        { swerve.heading.`in`(Degrees) }, swerve.maxSpeeds.times(0.75)
    )
    private val pathPlannerAutonomous = net.tecdroid.autonomous.PathPlannerAutonomous(swerve, limelightController, arm)
    private val swerveRotationLockSystem = SwerveRotationLockSystem(swerve, controller)
    private val reefAppListener = ReefAppListener()

    private val xLimelightToAprilTagSetPoint = 0.215
    private val yLimelightToAprilTagSetPoint = 0.045

    private var autoLevelSelectorMode = true

    // Advantage Scope log publisher
    private val robotPosePublisher: StructPublisher<Pose2d> = NetworkTableInstance.getDefault()
        .getStructTopic("RobotPose", Pose2d.struct).publish()

    init {
        limelightController.shuffleboardData()
        swerve.heading = 0.0.degrees

        arm.publishShuffleBoardData()
        arm.assignCommands()
    }


    fun autonomousInit() {
        swerve.removeDefaultCommand()
    }

    fun teleopInit() {
        controller.start().onTrue(swerve.zeroHeadingCommand())

        limelightController.setFilterIds(arrayOf(21, 20, 19, 18, 17, 22, 10, 11, 6, 7, 8, 9))

        swerve.defaultCommand = Commands.run(
            {
                val vx = MathUtil.applyDeadband(controller.leftY, 0.05) * 0.85
                val vy = MathUtil.applyDeadband(controller.leftX, 0.05) * 0.85
                val vw = MathUtil.applyDeadband(controller.rightX, 0.05) * 0.85

                val targetXVelocity = swerve.maxLinearVelocity * vx
                val targetYVelocity = swerve.maxLinearVelocity * vy
                val targetAngularVelocity = swerve.maxAngularVelocity * vw

                swerve.driveFieldOriented(ChassisSpeeds(targetXVelocity, targetYVelocity, targetAngularVelocity))
            },
            swerve
        )

        controller.rightTrigger().whileTrue(limelightController.alignRobotAllAxis(LimeLightChoice.Right, 0.215, 0.035))
        controller.leftTrigger().whileTrue(limelightController.alignRobotAllAxis(LimeLightChoice.Left, 0.215, -0.035))

        // Auto Level Selector

        controller.back().onTrue(Commands.runOnce({ autoLevelSelectorMode = !autoLevelSelectorMode }))

        controller.rightTrigger().and({ limeLightIsAtSetPoint(0.1, LimeLightChoice.Right) && autoLevelSelectorMode})
            .onTrue(Commands.runOnce({ betterLevelSequence(LimeLightChoice.Right) }))

        controller.leftTrigger().and({ limeLightIsAtSetPoint(0.1, LimeLightChoice.Left) && autoLevelSelectorMode})
            .onTrue(Commands.runOnce({ betterLevelSequence(LimeLightChoice.Left) }))

        //controller.rightTrigger().whileTrue(limelightController.alignRobotAllAxis({ reefAppListener.branchChoice.sideChoice }, 0.215, 0.035))
        //Trigger({ limeLightIsAtSetPoint(0.1, reefAppListener.branchChoice.sideChoice) }).onTrue(arm.scoringSequence({ reefAppListener.branchChoice.levelPose }))

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
        robotPosePublisher.set(swerve.pose)
    }

    fun limeLightIsAtSetPoint(xToleranceRange: Double = 0.0): Boolean {
        return limelightController.isAtSetPoint(LimeLightChoice.Right, xLimelightToAprilTagSetPoint, yLimelightToAprilTagSetPoint, xToleranceRange) ||
                limelightController.isAtSetPoint(LimeLightChoice.Left, xLimelightToAprilTagSetPoint, yLimelightToAprilTagSetPoint.unaryMinus(), xToleranceRange)
    }

    fun limeLightIsAtSetPoint(xToleranceRange: Double = 0.0, limeLightChoice: LimeLightChoice): Boolean {
        return when (limeLightChoice) {
            LimeLightChoice.Right -> limelightController.isAtSetPoint(LimeLightChoice.Right, xLimelightToAprilTagSetPoint, yLimelightToAprilTagSetPoint, xToleranceRange)
            LimeLightChoice.Left -> limelightController.isAtSetPoint(LimeLightChoice.Left, xLimelightToAprilTagSetPoint, yLimelightToAprilTagSetPoint.unaryMinus(), xToleranceRange)
        }
    }


    fun robotPeriodic() {
        advantageScopeLogs()
        try {
            limelightController.updatePoseLeftLimelight(swerve.poseEstimator)
        } catch (e: Exception) {
            System.out.println("left limelight pose update error")
        }

        try {
            limelightController.updatePoseRightLimelight(swerve.poseEstimator)
        } catch (e: Exception) {
            System.out.println("Right limelight update error")
        }

    }

    val autonomousCommand: Command
        get() = pathPlannerAutonomous.selectedAutonomousRoutine

}
