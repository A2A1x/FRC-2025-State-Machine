package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.reefscape.FieldConstants;
import frc.reefscape.FieldHelpers;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.util.Util;
import lombok.Getter;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

    // ------------------------------------------------------------------------
    // State machine
    // ------------------------------------------------------------------------
    public enum WantedState {
        TELEOP_DRIVE,
        ROTATION_LOCK,
        DRIVE_TO_POINT,
        IDLE
    }

    public enum SystemState {
        TELEOP_DRIVE,
        ROTATION_LOCK,
        DRIVE_TO_POINT,
        IDLE
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    private Rotation2d desiredRotation = Rotation2d.kZero;
    private Pose2d desiredPoseForDriveToPoint = new Pose2d();

    public static final double TRANSLATION_ERROR_MARGIN_METERS = Units.inchesToMeters(1.0);
    public static final double DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT = 0.02;

    private final PIDController teleopDriveToPointController = new PIDController(3.6, 0, 0.1);

    private double teleopVelocityCoefficient = 1.0;
    private double rotationVelocityCoefficient = 1.0;

    private static final double SKEW_COMPENSATION_SCALAR = -0.03;

    // ------------------------------------------------------------------------
    // Hardware / config
    // ------------------------------------------------------------------------
    @Getter private SwerveConfig config;
    private Notifier simNotifier = null;
    private double lastSimTime;

    // ------------------------------------------------------------------------
    // Controllers
    // ------------------------------------------------------------------------

    @Getter protected SwerveModuleState[] setpoints = new SwerveModuleState[] {};

    // ------------------------------------------------------------------------
    // CTRE requests
    // ------------------------------------------------------------------------
    private final SwerveRequest.ApplyRobotSpeeds PATHPLANNER_REQUEST =
            new SwerveRequest.ApplyRobotSpeeds();

    private static final SwerveRequest.ApplyFieldSpeeds FIELD_CENTRIC_DRIVE =
            new SwerveRequest.ApplyFieldSpeeds().withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.FieldCentricFacingAngle DRIVE_AT_ANGLE_REQUEST =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    // ------------------------------------------------------------------------
    // Constructor
    // ------------------------------------------------------------------------
    /**
     * Constructs a new Swerve drive subsystem.
     *
     * @param config The configuration object containing drivetrain constants and module
     *     configurations.
     */
    public Swerve(SwerveConfig config) {
        super(
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                config.getDrivetrainConstants(),
                config.getModules());
        this.config = config;
        configurePathPlanner();

        // Configure heading PID on the shared driveAtAngle request
        DRIVE_AT_ANGLE_REQUEST.HeadingController =
                new PhoenixPIDController(
                        config.getKPRotationController(),
                        config.getKIRotationController(),
                        config.getKDRotationController());
        DRIVE_AT_ANGLE_REQUEST.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        this.register();
        registerTelemetry(this::log);
        Telemetry.print(getName() + " Subsystem Initialized: ");
    }

    // ------------------------------------------------------------------------
    // Periodic / state machine
    // ------------------------------------------------------------------------
    @Override
    public void periodic() {

        systemState = handleStateTransition();
        applyStates();

        Telemetry.log("Swerve/SystemState", systemState.toString());
        Telemetry.log("Swerve/WantedState", wantedState.toString());
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case TELEOP_DRIVE -> SystemState.TELEOP_DRIVE;
            case ROTATION_LOCK -> SystemState.ROTATION_LOCK;
            case DRIVE_TO_POINT -> SystemState.DRIVE_TO_POINT;
            default -> SystemState.IDLE;
        };
    }

    private void applyStates() {
        switch (systemState) {
            default:
            case IDLE:
                break;

            case TELEOP_DRIVE:
                setControl(FIELD_CENTRIC_DRIVE.withSpeeds(calculateSpeedsBasedOnJoystickInputs()));
                break;

            case ROTATION_LOCK:
                setControl(
                        DRIVE_AT_ANGLE_REQUEST
                                .withVelocityX(
                                        calculateSpeedsBasedOnJoystickInputs().vxMetersPerSecond)
                                .withVelocityY(
                                        calculateSpeedsBasedOnJoystickInputs().vyMetersPerSecond)
                                .withTargetDirection(desiredRotation));
                break;

            case DRIVE_TO_POINT:
                {
                    var toTarget =
                            desiredPoseForDriveToPoint
                                    .getTranslation()
                                    .minus(getRobotPose().getTranslation());
                    double distance = toTarget.getNorm();
                    double friction =
                            distance >= Units.inchesToMeters(0.5)
                                    ? DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT
                                            * config.getSpeedAt12Volts().baseUnitMagnitude()
                                    : 0.0;
                    double speed =
                            Math.min(
                                    Math.abs(teleopDriveToPointController.calculate(distance, 0))
                                            + friction,
                                    config.getSpeedAt12Volts().baseUnitMagnitude());
                    var dir = toTarget.getAngle();

                    setControl(
                            DRIVE_AT_ANGLE_REQUEST
                                    .withVelocityX(speed * dir.getCos())
                                    .withVelocityY(speed * dir.getSin())
                                    .withTargetDirection(desiredPoseForDriveToPoint.getRotation()));
                    break;
                }
        }
    }

    private ChassisSpeeds calculateSpeedsBasedOnJoystickInputs() {
        if (DriverStation.getAlliance().isEmpty()) {
            return new ChassisSpeeds(0, 0, 0);
        }

        double xMagnitude = Robot.getPilot().getDriveFwdPositive();
        double yMagnitude = Robot.getPilot().getDriveLeftPositive();
        double angularMagnitude = Robot.getPilot().getDriveCCWPositive();
        angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

        double xVelocity =
                (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                                        == DriverStation.Alliance.Blue
                                ? xMagnitude
                                : -xMagnitude)
                        * teleopVelocityCoefficient;
        double yVelocity =
                (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                                        == DriverStation.Alliance.Blue
                                ? yMagnitude
                                : -yMagnitude)
                        * teleopVelocityCoefficient;
        double angularVelocity =
                (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                                        == DriverStation.Alliance.Blue
                                ? -angularMagnitude
                                : angularMagnitude)
                        * rotationVelocityCoefficient;

        Rotation2d skewCompensationFactor =
                Rotation2d.fromRadians(
                        getCurrentRobotChassisSpeeds().omegaRadiansPerSecond
                                * SKEW_COMPENSATION_SCALAR);

        return ChassisSpeeds.fromRobotRelativeSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(xVelocity, yVelocity, -angularVelocity),
                        getRobotPose().getRotation()),
                getRobotPose().getRotation().plus(skewCompensationFactor));
    }

    // ------------------------------------------------------------------------
    // Public state setters
    // ------------------------------------------------------------------------
    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    /**
     * Lock the robot's heading to the given rotation while the pilot steers translation.
     *
     * @param rotation Target heading
     */
    public void setTargetRotation(Rotation2d rotation) {
        this.desiredRotation = rotation;
        setWantedState(WantedState.ROTATION_LOCK);
    }

    /**
     * Drive to a field-relative pose, holding the pose's rotation with the heading controller.
     *
     * @param pose Target pose
     */
    public void setDesiredPoseForDriveToPoint(Pose2d pose) {
        this.desiredPoseForDriveToPoint = pose;
        setWantedState(WantedState.DRIVE_TO_POINT);
    }

    public void setTeleopVelocityCoefficient(double teleopVelocityCoefficient) {
        this.teleopVelocityCoefficient = teleopVelocityCoefficient;
    }

    public void setRotationVelocityCoefficient(double rotationVelocityCoefficient) {
        this.rotationVelocityCoefficient = rotationVelocityCoefficient;
    }

    public boolean isAtDriveToPointSetpoint() {
        double distance =
                desiredPoseForDriveToPoint
                        .getTranslation()
                        .minus(getRobotPose().getTranslation())
                        .getNorm();
        return distance < TRANSLATION_ERROR_MARGIN_METERS;
    }

    public boolean isAtDesiredRotation() {
        return isAtDesiredRotation(Units.degreesToRadians(10.0));
    }

    public boolean isAtDesiredRotation(double toleranceRadians) {
        return Math.abs(DRIVE_AT_ANGLE_REQUEST.HeadingController.getPositionError())
                < toleranceRadians;
    }

    // ------------------------------------------------------------------------
    // Pose / field helpers
    // ------------------------------------------------------------------------
    public Pose2d getRobotPose() {
        return keepPoseOnField(getState().Pose);
    }

    private Pose2d keepPoseOnField(Pose2d pose) {
        double halfRobot = config.getRobotLength() / 2;
        double newX = Util.limit(pose.getX(), halfRobot, FieldConstants.fieldLength - halfRobot);
        double newY = Util.limit(pose.getY(), halfRobot, FieldConstants.fieldWidth - halfRobot);
        if (pose.getX() != newX || pose.getY() != newY) {
            pose = new Pose2d(new Translation2d(newX, newY), pose.getRotation());
            resetPose(pose);
        }
        return pose;
    }

    public Trigger inXzone(double minXmeter, double maxXmeter) {
        return new Trigger(
                () -> Util.inRange(() -> getRobotPose().getX(), () -> minXmeter, () -> maxXmeter));
    }

    public Trigger inYzone(double minYmeter, double maxYmeter) {
        return new Trigger(
                () -> Util.inRange(() -> getRobotPose().getY(), () -> minYmeter, () -> maxYmeter));
    }

    /**
     * This method is used to check if the robot is in the X zone of the field flips the values if
     * Red Alliance
     *
     * @param minXmeter
     * @param maxXmeter
     * @return
     */
    public Trigger inXzoneAlliance(double minXmeter, double maxXmeter) {
        return new Trigger(
                () ->
                        Util.inRange(
                                FieldHelpers.flipXifRed(getRobotPose().getX()),
                                minXmeter,
                                maxXmeter));
    }

    /**
     * This method is used to check if the robot is in the Y zone of the field flips the values if
     * Red Alliance
     *
     * @param minYmeter
     * @param maxYmeter
     * @return
     */
    public Trigger inYzoneAlliance(double minYmeter, double maxYmeter) {
        return new Trigger(
                () ->
                        Util.inRange(
                                FieldHelpers.flipYifRed(getRobotPose().getY()),
                                minYmeter,
                                maxYmeter));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    // ------------------------------------------------------------------------
    // Rotation helpers
    // ------------------------------------------------------------------------
    Rotation2d getRotation() {
        return getRobotPose().getRotation();
    }

    double getRotationRadians() {
        return getRotation().getRadians();
    }

    protected void reorient(double angleDegrees) {
        resetPose(
                new Pose2d(
                        getRobotPose().getX(),
                        getRobotPose().getY(),
                        Rotation2d.fromDegrees(angleDegrees)));
    }

    protected Command reorientPilotAngle(double angleDegrees) {
        return runOnce(
                () ->
                        reorient(
                                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                                                == DriverStation.Alliance.Blue
                                        ? angleDegrees + 180
                                        : angleDegrees));
    }

    protected double getClosestCardinal() {
        double heading = getRotation().getRadians();
        if (heading > -Math.PI / 4 && heading <= Math.PI / 4) return 0;
        else if (heading > Math.PI / 4 && heading <= 3 * Math.PI / 4) return 90;
        else if (heading > 3 * Math.PI / 4 || heading <= -3 * Math.PI / 4) return 180;
        else return 270;
    }

    protected double getClosest45() {
        double angleDegrees = getRotation().getDegrees() % 360;
        if (angleDegrees < 0) angleDegrees += 360;
        return Rotation2d.fromDegrees(Math.round(angleDegrees / 45.0) * 45.0).getRadians();
    }

    protected double getClosestFieldAngle() {
        double angleDegrees = getRotation().getDegrees();
        double[] angleTable = {0, 180, 126, -126, 54, -54, 60, -60, 120, -120, 90, -90};
        double closestAngle = angleTable[0];
        double minDiff = getRotationDifference(angleDegrees, closestAngle);
        for (double angle : angleTable) {
            double diff = getRotationDifference(angleDegrees, angle);
            if (diff < minDiff) {
                minDiff = diff;
                closestAngle = angle;
            }
        }
        return Math.toRadians(closestAngle);
    }

    protected Command cardinalReorient() {
        return runOnce(() -> reorient(getClosestCardinal()));
    }

    public boolean frontClosestToAngle(double angleDegrees) {
        double heading = getRotation().getDegrees();
        double flippedHeading = heading > 0 ? heading - 180 : heading + 180;
        return getRotationDifference(heading, angleDegrees)
                < getRotationDifference(flippedHeading, angleDegrees);
    }

    public double getRotationDifference(double angle1, double angle2) {
        double diff = Math.abs(angle1 - angle2) % 360;
        return diff > 180 ? 360 - diff : diff;
    }

    // ------------------------------------------------------------------------
    // PathPlanner configuration
    // ------------------------------------------------------------------------
    private void configurePathPlanner() {
        resetPose(
                new Pose2d(
                        Units.feetToMeters(27.0),
                        Units.feetToMeters(27.0 / 2.0),
                        config.getBlueAlliancePerspectiveRotation()));

        RobotConfig robotConfig = null;
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                () -> this.getState().Pose,
                this::resetPose,
                this::getCurrentRobotChassisSpeeds,
                speeds -> this.setControl(PATHPLANNER_REQUEST.withSpeeds(speeds)),
                new PPHolonomicDriveController(
                        new PIDConstants(6, 0, 0), new PIDConstants(8, 0, 0), Robot.kDefaultPeriod),
                robotConfig,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
    }

    // ------------------------------------------------------------------------
    // Telemetry
    // ------------------------------------------------------------------------
    protected void log(SwerveDriveState state) {
        Telemetry.log("Swerve/Pose", state.Pose);
        Telemetry.log("Swerve/TargetStates", state.ModuleTargets);
        Telemetry.log("Swerve/MeasuredStates", state.ModuleStates);
        Telemetry.log("Swerve/MeasuredSpeeds", state.Speeds);
    }

    // ------------------------------------------------------------------------
    // Simulation
    // ------------------------------------------------------------------------
    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();
        simNotifier =
                new Notifier(
                        () -> {
                            final double currentTime = Utils.getCurrentTimeSeconds();
                            double deltaTime = currentTime - lastSimTime;
                            lastSimTime = currentTime;
                            updateSimState(deltaTime, RobotController.getBatteryVoltage());
                        });
        simNotifier.startPeriodic(config.getSimLoopPeriod());
    }
}
