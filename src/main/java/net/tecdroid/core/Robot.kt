package net.tecdroid.core

import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.BuildConstants
import net.tecdroid.constants.Constants
import net.tecdroid.constants.Constants.currentMode
import net.tecdroid.constants.SwerveTunerConstants
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter


class Robot : LoggedRobot {
    private val container = RobotContainer()
    private val autonomousCommand: Command
        get() = container.autonomousCommand

    constructor() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        when (BuildConstants.DIRTY) {
            0 -> Logger.recordMetadata("GitDirty", "All changes committed")
            1 -> Logger.recordMetadata("GitDirty", "Uncomitted changes")
            else -> Logger.recordMetadata("GitDirty", "Unknown")
        }

        when (currentMode) {
            Constants.Mode.REAL -> {
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4Publisher())
            }
            Constants.Mode.SIM -> { Logger.addDataReceiver(NT4Publisher()) }
            Constants.Mode.REPLAY -> {
                // Replaying a log, set up replay source
                setUseTiming(false) // Run as fast as possible
                val logPath = LogFileUtil.findReplayLog()
                Logger.setReplaySource(WPILOGReader(logPath))
                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")))
            }
        }

        // Start AdvantageKit logger
        //Logger.start() // Not Logging due to memory issues -roboRIO 1 apparently can't handle AK properly-.


        // Check for valid swerve config
        val modules =
            arrayOf<SwerveModuleConstants<*, *, *>?>(
                SwerveTunerConstants.FrontLeft,
                SwerveTunerConstants.FrontRight,
                SwerveTunerConstants.BackLeft,
                SwerveTunerConstants.BackRight
            )
        for (constants in modules) {
            if (constants!!.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
                || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated
            ) {
                throw RuntimeException(
                    "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers."
                )
            }
        }
    }

    override fun robotInit() {
        DriverStation.silenceJoystickConnectionWarning(true)
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        container.robotPeriodic()
    }

    override fun disabledInit() {
        CommandScheduler.getInstance().cancelAll()
        container.disableInit()
    }

    override fun disabledPeriodic() {
    }

    override fun disabledExit() {
    }

    override fun autonomousInit() {
        container.autonomousInit()
        autonomousCommand.schedule()
    }

    override fun autonomousPeriodic() {
    }

    override fun teleopInit() {
        container.teleopInit()
        if (autonomousCommand.isScheduled) autonomousCommand.cancel()
    }

    override fun teleopPeriodic() {
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testPeriodic() {
    }

    override fun simulationInit() {
    }

    override fun simulationPeriodic() {
    }
}
