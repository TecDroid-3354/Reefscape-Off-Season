package net.tecdroid.vision.limelight.systems;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers;
import net.tecdroid.constants.StringConstantsKt;
import net.tecdroid.systems.ArmSystem.*;
import net.tecdroid.util.ControlGains;
import net.tecdroid.vision.limelight.Limelight;
import net.tecdroid.vision.limelight.LimelightAprilTagDetector;
import net.tecdroid.vision.limelight.LimelightConfig;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

record ReefPosesForwardDelta(Distance backL4Delta, Distance backL3Delta, Distance backL2Delta){}

public class LimelightController {
    private final LimelightAprilTagDetector leftLimelight  = new LimelightAprilTagDetector(new LimelightConfig(StringConstantsKt.leftLimelightName, new Pose3d()));
    private final LimelightAprilTagDetector rightLimelight = new LimelightAprilTagDetector(new LimelightConfig(StringConstantsKt.rightLimelightName, new Pose3d()));

    /** Setpoints for the back left camera as follows: Pair<FrontalDistance, HorizontalDistance(As closer to zero, distance will increase)> */
    private final Pair<Distance, Distance> backLeftLLSetpoints = new Pair<>(Centimeters.of(33.5), Centimeters.of(2.5));
    /** Setpoints for the back right camera as follows: Pair<FrontalDistance, HorizontalDistance> */
    private final Pair<Distance, Distance> backRightLLSetpoints = new Pair<>(Centimeters.of(33.5), Centimeters.of(-2.5));
    private final Distance positionTolerance = Centimeters.of(1.5);

    /** Chassis: 27.5in * 27.5in; Center: 27.5in / 2 = 13.75in; Bumpers = 3.25in
     * back LL: LLForward = -0.1905m; back LL lens to end of bumper: (Center + Bumpers) - LLForward = 24.13cm
     * frontLL: LLForward = 0.635cm; front LL lens to end of bumper: (Center + Bumpers) - LLForward = 42.545cm
     *
     * Process sample for calculating deltas:
     * backL4Delta: 24.13cm + 1in (From Design) = 26.67cm -> 31.5cm (current forward setpoint) - 26.67cm = 4.83cm
     * ...
     * TODO() = Recalculate base forward setpoint in front limelight
     */
    private final ReefPosesForwardDelta reefPosesForwardDeltas = new ReefPosesForwardDelta(
            Centimeters.of(-4.50), Centimeters.of(-4.83), Centimeters.of(0.25));
    private final Subsystem requiredSubsystem;

    private final ControlGains xyGains = new ControlGains(0.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    private final ControlGains thetaGains = new ControlGains(0.0075, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    private final PIDController xyBackPidController = new PIDController(xyGains.getP(), xyGains.getI(), xyGains.getD());
    private final PIDController thetaBackPidController = new PIDController(thetaGains.getP(), thetaGains.getI(), thetaGains.getD());

    private final Map<Integer, Double> backLLAlignmentAngles = new HashMap<>();
    private final Consumer<ChassisSpeeds> drive;
    private final DoubleSupplier yaw;

    private final ChassisSpeeds maxSpeeds;

    private HttpCamera leftStream = new HttpCamera("lll", "http://10.33.54.201:5800");
    private HttpCamera rightStream = new HttpCamera("llr", "http://10.33.54.203:5800");

    private void angleDictionaryValues() {
        // Blue
        backLLAlignmentAngles.put(21, 0.0);
        backLLAlignmentAngles.put(20, 60.0);
        backLLAlignmentAngles.put(19, 120.0);
        backLLAlignmentAngles.put(18, 180.0);
        backLLAlignmentAngles.put(17, 240.0);
        backLLAlignmentAngles.put(22, 300.0);

        // Red
        backLLAlignmentAngles.put(10, 0.0);
        backLLAlignmentAngles.put(11, 60.0);
        backLLAlignmentAngles.put(6, 120.0);
        backLLAlignmentAngles.put(7, 180.0);
        backLLAlignmentAngles.put(8, 240.0);
        backLLAlignmentAngles.put(9, 300.0);
    }

    private void limelightConfiguration() {
        thetaBackPidController.enableContinuousInput(0.0, 360.0);

        Integer[] validIDs = { 21, 20, 19, 18, 17, 22, 10, 11, 6, 7, 8, 9 };
        rightLimelight.setIdFilter(validIDs);
        leftLimelight.setIdFilter(validIDs);

    }

    public void setThrottle(int throttle) {
        leftLimelight.setThrottle(throttle);
        rightLimelight.setThrottle(throttle);
    }

    public void limelightsStream() {
        ShuffleboardTab driverTab = Shuffleboard.getTab("Driver Tab");

        try {
            MjpegServer leftServer = CameraServer.startAutomaticCapture(leftStream);
            MjpegServer rightServer = CameraServer.startAutomaticCapture(rightStream);

            driverTab.add("Left Limelight", leftStream);
            driverTab.add("Right Limelight", rightStream);

        } catch (Exception ignored) {}
    }

    public LimelightController(Subsystem requiredSubsystem, Consumer<ChassisSpeeds> drive, DoubleSupplier yaw, ChassisSpeeds maxSpeeds) {
        this.requiredSubsystem = requiredSubsystem;
        this.drive = drive;
        this.yaw = yaw;
        this.maxSpeeds = maxSpeeds;

        angleDictionaryValues();
        limelightConfiguration();
        limelightsStream();
    }

    private static double clamp(double max, double min, double v) {
        return Math.max(min, Math.min(max, v));
    }

    public boolean isAtSetPoint(LimeLightChoice choice, Pair<Distance, Distance> setpoints) {
        Pose3d robotPose = getTargetPositionInCameraSpace(choice);

        double xDisplacement = Math.abs(setpoints.getFirst().in(Meters) - robotPose.getTranslation().getZ());
        double yDisplacement = Math.abs(setpoints.getSecond().in(Meters) - robotPose.getTranslation().getX());

        return hasTarget(choice) && (xDisplacement <= positionTolerance.in(Meters) && yDisplacement <= positionTolerance.in(Meters));
    }

    public boolean isAtSetPoint(LimeLightChoice choice, Pair<Distance, Distance> setpoints, Distance tolerance) {
        Pose3d robotPose = getTargetPositionInCameraSpace(choice);

        double xDisplacement = Math.abs(setpoints.getFirst().in(Meters) - robotPose.getTranslation().getZ());
        double yDisplacement = Math.abs(setpoints.getSecond().in(Meters) - robotPose.getTranslation().getX());

        return hasTarget(choice) && (xDisplacement <= tolerance.in(Meters)) && yDisplacement <= tolerance.in(Meters);
    }
    public Pair<Distance, Distance> getRightLLSetpoints(ArmPoses pose) {
        Distance horizontalOffset = backRightLLSetpoints.getSecond();
        Distance forwardSetpoint = backRightLLSetpoints.getFirst();

        Distance forwardOffset = switch (pose) {
            case BackL2 -> forwardSetpoint.plus(Centimeters.of(6.0)); // TODO() = Match the 8.5 of left?
            default -> forwardSetpoint;
        };

        return new Pair<Distance, Distance>(forwardOffset, horizontalOffset);//new Pair<Distance, Distance>(forwardOffset, horizontalOffset);
    }

    public Pair<Distance, Distance> getLeftLLSetpoints(ArmPoses pose) {
        Distance horizontalOffset = backLeftLLSetpoints.getSecond();
        Distance forwardSetpoint = backLeftLLSetpoints.getFirst();

        Distance forwardOffset = switch (pose) {
            case BackL2 -> forwardSetpoint.plus(Centimeters.of(8.5));
            default -> forwardSetpoint;
        };
        return new Pair<Distance, Distance>(forwardOffset, horizontalOffset);
    }

    private Pose3d getTargetPositionInCameraSpace(LimeLightChoice choice) {
        LimelightAprilTagDetector limelight = (choice == LimeLightChoice.Right) ? rightLimelight : leftLimelight;
        return limelight.getTargetPositionInCameraSpace();
    }

    public int getTargetId(LimeLightChoice choice) {
        LimelightAprilTagDetector limelight = (choice == LimeLightChoice.Right) ? rightLimelight : leftLimelight;
        return limelight.getTargetId();
    }

    private PIDController getXYPidController(LimeLightChoice choice) {
        return xyBackPidController;
    }

    private PIDController getThetaPIDController(LimeLightChoice choice) {
        return thetaBackPidController;
    }

    public boolean hasTarget(LimeLightChoice choice) {
        Limelight limelight = (choice == LimeLightChoice.Right) ? rightLimelight : leftLimelight;
        return limelight.getHasTarget();
    }

    public void setFilterIds(Integer[] targetIds) {
        rightLimelight.setIdFilter(targetIds);
        leftLimelight.setIdFilter(targetIds);
    }
    // Obtain a yaw between the range [0, 360]

    private double getLimitedYaw() {
        double limitedYaw = yaw.getAsDouble() % 360;
        if (limitedYaw < 0) {
            limitedYaw += 360;
        }
        return limitedYaw;
    }

    public void alignRobotAllAxisAuto(Supplier<LimeLightChoice> choice, Supplier<Pair<Distance, Distance>> setpoints) {
            int id = getTargetId(choice.get());

            if (!hasTarget(choice.get()) || !backLLAlignmentAngles.containsKey(id) || isAtSetPoint(choice.get(), setpoints.get())) {
                drive.accept(new ChassisSpeeds(0.0, 0.0, 0.0));
                return;
            }

            Pose3d robotPose = getTargetPositionInCameraSpace(choice.get());
            PIDController xyPIDController = getXYPidController(choice.get());
            PIDController thetaPIDController = getThetaPIDController(choice.get());
            double targetAngleDegrees = backLLAlignmentAngles.get(id);

            xyPIDController.reset();
            thetaPIDController.reset();
            double xFactor = clamp(1.0, -1.0, xyPIDController.calculate(robotPose.getTranslation().getZ(), setpoints.get().getFirst().in(Meters)));
            double yFactor = clamp(1.0, -1.0, xyPIDController.calculate(robotPose.getTranslation().getX(), setpoints.get().getSecond().in(Meters)));
            double wFactor = clamp(1.0, -1.0, thetaPIDController.calculate(getLimitedYaw(), targetAngleDegrees));

            LinearVelocity xVelocity = MetersPerSecond.of(maxSpeeds.vxMetersPerSecond * xFactor);
            LinearVelocity yVelocity = MetersPerSecond.of(maxSpeeds.vyMetersPerSecond * yFactor);
            AngularVelocity wVelocity = DegreesPerSecond.of(Math.toDegrees(maxSpeeds.omegaRadiansPerSecond) * wFactor);

            drive.accept(new ChassisSpeeds(xVelocity, yVelocity, wVelocity));
    }
    public Command alignRobotAllAxis(Supplier<LimeLightChoice> choice, Supplier<Pair<Distance, Distance>> setpoints) {
        return Commands.run(() -> {
            int id = getTargetId(choice.get());

            if (!hasTarget(choice.get()) || !backLLAlignmentAngles.containsKey(id) || isAtSetPoint(choice.get(), setpoints.get())) {
                drive.accept(new ChassisSpeeds(0.0, 0.0, 0.0));
                return;
            }

            Pose3d robotPose = getTargetPositionInCameraSpace(choice.get());
            PIDController xyPIDController = getXYPidController(choice.get());
            PIDController thetaPIDController = getThetaPIDController(choice.get());
            double targetAngleDegrees = backLLAlignmentAngles.get(id);

            xyPIDController.reset();
            thetaPIDController.reset();
            double xFactor = clamp(1.0, -1.0, xyPIDController.calculate(robotPose.getTranslation().getZ(), setpoints.get().getFirst().in(Meters)));
            double yFactor = clamp(1.0, -1.0, xyPIDController.calculate(robotPose.getTranslation().getX(), setpoints.get().getSecond().in(Meters)));
            double wFactor = clamp(1.0, -1.0, thetaPIDController.calculate(getLimitedYaw(), targetAngleDegrees));

            LinearVelocity xVelocity = MetersPerSecond.of(maxSpeeds.vxMetersPerSecond * xFactor);
            LinearVelocity yVelocity = MetersPerSecond.of(maxSpeeds.vyMetersPerSecond * yFactor);
            AngularVelocity wVelocity = DegreesPerSecond.of(Math.toDegrees(maxSpeeds.omegaRadiansPerSecond) * wFactor);

            drive.accept(new ChassisSpeeds(xVelocity, yVelocity, wVelocity));

        }, requiredSubsystem);
    }

    public Command alignRobotAllAxis(Supplier<LimeLightChoice> choice, Supplier<Pair<Distance, Distance>> setpoints, Double extraDegreesSetpoint) {
        return Commands.run(() -> {
            int id = getTargetId(choice.get());

            if (!hasTarget(choice.get()) || !backLLAlignmentAngles.containsKey(id) || isAtSetPoint(choice.get(), setpoints.get())) {
                drive.accept(new ChassisSpeeds(0.0, 0.0, 0.0));
                return;
            }

            Pose3d robotPose = getTargetPositionInCameraSpace(choice.get());
            PIDController xyPIDController = getXYPidController(choice.get());
            PIDController thetaPIDController = getThetaPIDController(choice.get());
            double targetAngleDegrees = backLLAlignmentAngles.get(id);

            xyPIDController.reset();
            thetaPIDController.reset();
            double xFactor = clamp(1.0, -1.0, xyPIDController.calculate(robotPose.getTranslation().getZ(), setpoints.get().getFirst().in(Meters)));
            double yFactor = clamp(1.0, -1.0, xyPIDController.calculate(robotPose.getTranslation().getX(), setpoints.get().getSecond().in(Meters)));
            double wFactor = clamp(1.0, -1.0, thetaPIDController.calculate(getLimitedYaw(), targetAngleDegrees - extraDegreesSetpoint));

            LinearVelocity xVelocity = MetersPerSecond.of(maxSpeeds.vxMetersPerSecond * xFactor);
            LinearVelocity yVelocity = MetersPerSecond.of(maxSpeeds.vyMetersPerSecond * yFactor);
            AngularVelocity wVelocity = DegreesPerSecond.of(Math.toDegrees(maxSpeeds.omegaRadiansPerSecond) * wFactor);

            drive.accept(new ChassisSpeeds(xVelocity, yVelocity, wVelocity));

        }, requiredSubsystem);
    }

    public Command alignRobotAllAxis(Supplier<LimeLightChoice> limelightChoice, Pair<Distance, Distance> setpoints) {
        return Commands.run(() -> {
            LimeLightChoice choice = limelightChoice.get();

            int id = getTargetId(choice);

            if (!hasTarget(choice) || !backLLAlignmentAngles.containsKey(id) || isAtSetPoint(choice, setpoints)) {
                drive.accept(new ChassisSpeeds(0.0, 0.0, 0.0));
                return;
            }

            Pose3d robotPose = getTargetPositionInCameraSpace(choice);
            PIDController xyPIDController = getXYPidController(choice);
            PIDController thetaPIDController = getThetaPIDController(choice);
            double targetAngleDegrees = backLLAlignmentAngles.get(id);

            xyPIDController.reset();
            thetaPIDController.reset();
            double xFactor = clamp(1.0, -1.0, xyPIDController.calculate(robotPose.getTranslation().getZ(), setpoints.getFirst().in(Meters)));
            double yFactor = clamp(1.0, -1.0, xyPIDController.calculate(robotPose.getTranslation().getX(), setpoints.getSecond().in(Meters)));
            double wFactor = clamp(1.0, -1.0, thetaPIDController.calculate(getLimitedYaw(), targetAngleDegrees));

            LinearVelocity xVelocity = MetersPerSecond.of(maxSpeeds.vxMetersPerSecond * xFactor);
            LinearVelocity yVelocity = MetersPerSecond.of(maxSpeeds.vyMetersPerSecond * yFactor);
            AngularVelocity wVelocity = DegreesPerSecond.of(Math.toDegrees(maxSpeeds.omegaRadiansPerSecond) * wFactor);

            drive.accept(new ChassisSpeeds(xVelocity, yVelocity, wVelocity));

        }, requiredSubsystem);
    }

    public LimelightHelpers.PoseEstimate getRobotPoseEstimate(LimeLightChoice choice) {
        LimelightAprilTagDetector limelight = (choice == LimeLightChoice.Right) ? rightLimelight : leftLimelight;
        return limelight.getRobotPoseEstimateWpiBlueMT1();
    }

    public void updatePoseLeftLimelight(SwerveDrivePoseEstimator poseEstimator) {
        boolean doRejectUpdate = false;
        LimelightHelpers.PoseEstimate mt1 = leftLimelight.getRobotPoseEstimateWpiBlueMT1();

        if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
        {
            if(mt1.rawFiducials[0].ambiguity > .7)
            {
                doRejectUpdate = true;
            }
            if(mt1.rawFiducials[0].distToCamera > 3)
            {
                doRejectUpdate = true;
            }
        }

        if(mt1.tagCount == 0 || getTargetPositionInCameraSpace(LimeLightChoice.Left).getZ() > 1.5)
        {
            doRejectUpdate = true;
        }

        if(!doRejectUpdate)
        {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.0, 0.0, 0.0));
            poseEstimator.addVisionMeasurement(
                    mt1.pose,
                    mt1.timestampSeconds);
        }
    }

    public void updatePoseRightLimelight(SwerveDrivePoseEstimator poseEstimator) {
        boolean doRejectUpdate = false;
        LimelightHelpers.PoseEstimate mt1 = rightLimelight.getRobotPoseEstimateWpiBlueMT1();

        if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
        {
            if(mt1.rawFiducials[0].ambiguity > .7)
            {
                doRejectUpdate = true;
            }
            if(mt1.rawFiducials[0].distToCamera > 3)
            {
                doRejectUpdate = true;
            }
        }

        if(mt1.tagCount == 0 || getTargetPositionInCameraSpace(LimeLightChoice.Right).getZ() > 1.5)
        {
            doRejectUpdate = true;
        }

        if(!doRejectUpdate)
        {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.0, 0.0, 0.0));
            poseEstimator.addVisionMeasurement(
                    mt1.pose,
                    mt1.timestampSeconds);
        }
    }

    public void updatePoseMT2(SwerveDrivePoseEstimator poseEstimator, StatusSignal<AngularVelocity> angularVelocity, Rotation2d robotYaw) {
        boolean doRejectUpdate = false;

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(StringConstantsKt.leftLimelightName);
        LimelightHelpers.SetRobotOrientation(StringConstantsKt.leftLimelightName, poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);

        if(Math.abs(angularVelocity.getValueAsDouble()) > 360 || mt2.tagCount == 0) {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            //poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            poseEstimator.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);

        }
    }

    public void shuffleboardData() {
        ShuffleboardTab tab = Shuffleboard.getTab("Limelight");

        tab.addDoubleArray("rPosition", () -> new double[]{ rightLimelight.getRobotPoseEstimateWpiBlueMT1().pose.getX(),
                rightLimelight.getRobotPoseEstimateWpiBlueMT1().pose.getY(),
                rightLimelight.getRobotPoseEstimateWpiBlueMT1().pose.getRotation().getDegrees()});

        tab.addDoubleArray("lPosition", () -> new double[]{ leftLimelight.getRobotPoseEstimateWpiBlueMT1().pose.getX(),
                leftLimelight.getRobotPoseEstimateWpiBlueMT1().pose.getY(),
                leftLimelight.getRobotPoseEstimateWpiBlueMT1().pose.getRotation().getDegrees()});

        tab.addDouble("RobotYaw", this::getLimitedYaw);
        tab.addInteger("Yaw Objective", rightLimelight::getTargetId);
    }
}