package frc.robot.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.reefscape.Field;
import frc.reefscape.FieldHelpers;
import frc.reefscape.TagProperties;
import frc.reefscape.Zones;
import frc.reefscape.offsets.HomeOffsets;
import frc.robot.Robot;
import frc.robot.pilot.Pilot;
import frc.spectrumLib.SpectrumState;
import frc.spectrumLib.Telemetry;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Set;
import java.util.function.DoubleSupplier;

public class SwerveStates {

    // ------------------------------------------------------------------------
    // Dependencies / singletons
    // ------------------------------------------------------------------------
    static Swerve swerve = Robot.getSwerve();
    static SwerveConfig config = Robot.getConfig().swerve;
    static Pilot pilot = Robot.getPilot();
    static Zones zones = new Zones();

    // ------------------------------------------------------------------------
    // Requests (pre-configured CTRE swerve requests)
    // ------------------------------------------------------------------------
    private static final SwerveRequest.FieldCentric FIELD_CENTRIC_DRIVE =
            new SwerveRequest.FieldCentric()
                    .withDeadband(
                            config.getSpeedAt12Volts().in(MetersPerSecond) * config.getDeadband())
                    .withRotationalDeadband(config.getMaxAngularRate() * config.getDeadband())
                    .withDriveRequestType(DriveRequestType.Velocity);

    private static final SwerveRequest.RobotCentric ROBOT_CENTRIC_DRIVE =
            new SwerveRequest.RobotCentric()
                    .withDeadband(
                            config.getSpeedAt12Volts().in(MetersPerSecond) * config.getDeadband())
                    .withRotationalDeadband(config.getMaxAngularRate() * config.getDeadband())
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final SwerveRequest.SwerveDriveBrake X_BRAKE_REQUEST =
            new SwerveRequest.SwerveDriveBrake();

    private static final SwerveRequest.FieldCentricFacingAngle FIELD_CENTRIC_FACING_ANGLE =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(
                            config.getSpeedAt12Volts().in(MetersPerSecond)
                                    * config.getAimDeadband())
                    .withRotationalDeadband(config.getMaxAngularRate() * config.getAimDeadband())
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.Position)
                    .withMaxAbsRotationalRate(config.getMaxAngularRate())
                    .withHeadingPID(
                            config.getKPRotationController(),
                            config.getKIRotationController(),
                            config.getKDRotationController());

    private static final SwerveRequest.RobotCentricFacingAngle ROBOT_CENTRIC_FACING_ANGLE =
            new SwerveRequest.RobotCentricFacingAngle()
                    .withDeadband(
                            config.getSpeedAt12Volts().in(MetersPerSecond)
                                    * config.getAimDeadband())
                    .withRotationalDeadband(config.getMaxAngularRate() * config.getAimDeadband())
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.Position)
                    .withMaxAbsRotationalRate(config.getMaxAngularRate())
                    .withHeadingPID(
                            config.getKPRotationController(),
                            config.getKIRotationController(),
                            config.getKDRotationController());

    // ------------------------------------------------------------------------
    // Triggers
    // ------------------------------------------------------------------------

    public static final Trigger isFrontClosestToLeftStation =
            new Trigger(
                    () ->
                            swerve.frontClosestToAngle(
                                    FieldHelpers.flipAngleIfRed(
                                            Field.CoralStation.leftFaceRobotPovDegrees)));

    public static final Trigger isFrontClosestToRightStation =
            new Trigger(
                    () ->
                            swerve.frontClosestToAngle(
                                    FieldHelpers.flipAngleIfRed(
                                            Field.CoralStation.rightFaceRobotPovDegrees)));

    public static final Trigger isFrontClosestToNet =
            new Trigger(
                    () ->
                            swerve.frontClosestToAngle(Field.Barge.netRobotPovDegrees)
                                    == Zones.blueFieldSide.getAsBoolean());

    public static final Trigger isAlignedToReef = new Trigger(() -> Zones.atReef());

    // ------------------------------------------------------------------------
    // Default command
    // ------------------------------------------------------------------------
    static Command pilotSteerCommand =
            log(pilotDrive().withName("SwerveCommands.pilotSteer").ignoringDisable(true));

    static SpectrumState steeringLock = new SpectrumState("SteeringLock");

    protected static void setupDefaultCommand() {
        swerve.setDefaultCommand(pilotSteerCommand);
    }

    // ------------------------------------------------------------------------
    // Bindings / state setup
    // ------------------------------------------------------------------------
    protected static void setStates() {
        // Force back to manual steering when we steer
        pilot.steer.whileTrue(swerve.getDefaultCommand());

        pilot.fpv_LS.whileTrue(log(fpvDrive()));

        // Reorient bindings
        pilot.upReorient.onTrue(log(reorientForward()));
        pilot.leftReorient.onTrue(log(reorientLeft()));
        pilot.downReorient.onTrue(log(reorientBack()));
        pilot.rightReorient.onTrue(log(reorientRight()));

        // Vision / reef alignment bindings
        pilot.reefVision_A.whileTrue(log(reefAimDriveVisionXY()));
        pilot.reefAlignScore_B.whileTrue(log(reefAimDriveVisionXY()));
    }

    // ------------------------------------------------------------------------
    // Pilot commands
    // ------------------------------------------------------------------------

    /**
     * Drive the robot using left stick and control orientation using the right stick.
     *
     * @return A command that drives the robot with translation control from the left stick and
     *     rotation control from the right stick
     */
    protected static Command pilotDrive() {
        return drive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::getDriveCCWPositive)
                .withName("Swerve.PilotDrive");
    }
    /**
     * Drive the robot with its front bumper as the forward direction (first-person view).
     *
     * @return A command that drives the robot with the front bumper as the forward direction
     */
    protected static Command fpvDrive() {
        return fpvDrive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::getDriveCCWPositive)
                .withName("Swerve.PilotFPVDrive");
    }

    /**
     * Drive the robot with the robot's orientation snapping to the closest cardinal direction.
     *
     * @return A command that drives the robot with cardinal snapping
     */
    protected static Command snapSteerDrive() {
        return drive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::chooseCardinalDirections)
                .withName("Swerve.PilotStickSteer");
    }

    /**
     * Drive the robot with the front bumper trying to match the robot's motion direction.
     *
     * @return A command that drives the robot while matching motion direction with robot angle
     */
    protected static Command snakeDrive() {
        return aimDrive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::getPilotStickAngle)
                .withName("Swerve.SnakeDrive");
    }

    /**
     * Tweak the robot's orientation by a small angle back and forth.
     *
     * @return A command that repeatedly tweaks the robot's orientation
     */
    protected static Command tweakOut() {
        return Commands.defer(
                        () -> {
                            final double base = swerve.getRotation().getRadians();
                            final double delta = Math.toRadians(10.0);

                            Command toMinus =
                                    aimDrive(
                                                    pilot::getDriveFwdPositive,
                                                    pilot::getDriveLeftPositive,
                                                    () -> base - delta)
                                            .withTimeout(0.5);

                            Command toPlus =
                                    aimDrive(
                                                    pilot::getDriveFwdPositive,
                                                    pilot::getDriveLeftPositive,
                                                    () -> base + delta)
                                            .withTimeout(0.5);

                            return Commands.repeatingSequence(toMinus, toPlus);
                        },
                        Set.of(swerve))
                .withName("Swerve.tweakOut");
    }

    /** Turn the swerve wheels to an X to prevent the robot from moving. */
    protected static Command xBrake() {
        return swerve.applyRequest(() -> X_BRAKE_REQUEST).withName("Swerve.Xbrake");
    }

    /**
     * Drive the robot with the front bumper trying to match a target angle.
     *
     * @param targetDegrees The target angle supplier (expected in radians)
     * @return A command that drives the robot to match the target angle while allowing translation
     *     control with the left stick
     */
    protected static Command pilotAimDrive(DoubleSupplier targetDegrees) {
        return aimDrive(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive, targetDegrees)
                .withName("Swerve.PilotAimDrive");
    }

    // ------------------------------------------------------------------------
    // Reef / vision alignment commands
    // ------------------------------------------------------------------------

    public static Command autonAlgaeDriveIntake(double timeout) {
        return fpvAimDrive(() -> 0.75, () -> 0, FieldHelpers::getReefTagAngle).withTimeout(timeout);
    }

    public static Command reefAimDriveVisionTA() {
        return fpvAimDrive(
                        SwerveStates::getTagDistanceVelocity,
                        SwerveStates::getTagTxVelocity,
                        () -> FieldHelpers.getReefTagAngle())
                .withName("Swerve.reefAimDriveVisionTA");
    }

    public static Command autonAlgaeReefAimDriveVisionXY() {
        return alignDrive(
                        () -> FieldHelpers.getReefOffsetFromTagAlgaeX(),
                        () -> FieldHelpers.getReefOffsetFromTagAlgaeY(),
                        () -> FieldHelpers.getReefTagAngle())
                .withName("Swerve.autonAlgaeReefAimDriveVisionXY");
    }

    public static Command reefAimDriveVisionXY() {
        return alignDrive(
                        () -> FieldHelpers.getReefOffsetFromTagX(),
                        () -> FieldHelpers.getReefOffsetFromTagY(),
                        () -> FieldHelpers.getReefTagAngle())
                .withName("Swerve.reefAimDriveVisionXY");
    }

    public static Command reefAimDrive() {
        return alignDrive(
                        () -> zones.getScoreReefPoseX(),
                        () -> zones.getScoreReefPoseY(),
                        () -> zones.getScoreReefPoseAngle())
                .withName("Swerve.reefAimDrive");
    }

    public static Command alignToXDrive(DoubleSupplier xGoalMeters) {
        return resetXController()
                .andThen(
                        drive(
                                getAlignToX(xGoalMeters),
                                pilot::getDriveLeftPositive,
                                pilot::getDriveCCWPositive));
    }

    public static Command alignToYDrive(DoubleSupplier yGoalMeters) {
        return resetYController()
                .andThen(
                        drive(
                                pilot::getDriveFwdPositive,
                                getAlignToY(yGoalMeters),
                                pilot::getDriveCCWPositive));
    }

    public static Command alignXYDrive(DoubleSupplier xGoalMeters, DoubleSupplier yGoalMeters) {
        return resetXController()
                .alongWith(resetYController())
                .andThen(
                        drive(
                                getAlignToX(xGoalMeters),
                                getAlignToY(yGoalMeters),
                                pilot::getDriveCCWPositive));
    }

    /**
     * Align the robot to the given x, y, and heading goals.
     *
     * @param xGoalMeters The x goal in meters
     * @param yGoalMeters The y goal in meters
     * @param headingRadians The heading goal in radians
     * @return A command that aligns the robot to the specified x, y, and heading goals
     */
    public static Command alignDrive(
            DoubleSupplier xGoalMeters, DoubleSupplier yGoalMeters, DoubleSupplier headingRadians) {
        final boolean invertForRed = Field.isRed();

        return resetXController()
                .andThen(
                        resetYController(),
                        aimDrive(
                                () ->
                                        invertForRed
                                                ? -getAlignToX(xGoalMeters).getAsDouble()
                                                : getAlignToX(xGoalMeters).getAsDouble(),
                                () ->
                                        invertForRed
                                                ? -getAlignToY(yGoalMeters).getAsDouble()
                                                : getAlignToY(yGoalMeters).getAsDouble(),
                                headingRadians));
    }

    protected static Command headingLockDrive() {
        return headingLock(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive)
                .withName("Swerve.PilotHeadingLockDrive");
    }

    protected static Command lockToClosest45Drive() {
        return lockToClosest45deg(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive)
                .withName("Swerve.PilotLockTo45degDrive");
    }

    protected static Command lockToClosestFieldAngleDrive() {
        return lockToClosestFieldAngle(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive)
                .withName("Swerve.PilotLockToFieldAngleDrive");
    }

    // ------------------------------------------------------------------------
    // Vision velocity helpers
    // ------------------------------------------------------------------------
    private static double getTagTxVelocity() {
        if (Robot.getVision().tagsInView()) {
            return swerve.calculateTagCenterAlignController(
                    () -> 0, () -> Robot.getVision().getTagTX());
        }
        return 0;
    }

    private static double getTagDistanceVelocity() {
        TagProperties[] tagAreaOffsets = HomeOffsets.getReefTagOffsets();
        int tagIndex = Robot.getVision().getClosestTagID();
        if (tagIndex < 0) {
            return 0.0;
        } else if (tagIndex >= 17) {
            tagIndex -= 17;
        }
        final double tagAreaOffset = tagAreaOffsets[tagIndex].getTaGoal();
        SmartDashboard.putNumber("Tag Area Offset: ", tagAreaOffset);
        return swerve.calculateTagDistanceAlignController(() -> tagAreaOffset);
    }

    // ------------------------------------------------------------------------
    // Controller/heading helpers
    // ------------------------------------------------------------------------
    private static DoubleSupplier getAlignToX(DoubleSupplier xGoalMeters) {
        return swerve.calculateXController(xGoalMeters);
    }

    private static DoubleSupplier getAlignToY(DoubleSupplier yGoalMeters) {
        return swerve.calculateYController(yGoalMeters);
    }

    // ------------------------------------------------------------------------
    // Small helper commands (reset/set)
    // ------------------------------------------------------------------------
    protected static Command resetXController() {
        return swerve.runOnce(swerve::resetXController).withName("ResetXController");
    }

    protected static Command resetYController() {
        return swerve.runOnce(swerve::resetYController).withName("ResetYController");
    }

    // ------------------------------------------------------------------------
    // Core drive primitives (compose these into higher-level behaviors)
    // ------------------------------------------------------------------------
    private static Command drive(
            DoubleSupplier fwdPositive, DoubleSupplier leftPositive, DoubleSupplier ccwPositive) {
        return Commands.run(
                        () -> swerve.driveFieldRelative(fwdPositive, leftPositive, ccwPositive),
                        swerve)
                .withName("Swerve.drive");
    }

    private static Command fpvDrive(
            DoubleSupplier fwdPositive, DoubleSupplier leftPositive, DoubleSupplier ccwPositive) {
        return swerve.applyRequest(
                        () ->
                                ROBOT_CENTRIC_DRIVE
                                        .withVelocityX(fwdPositive.getAsDouble())
                                        .withVelocityY(leftPositive.getAsDouble())
                                        .withRotationalRate(ccwPositive.getAsDouble()))
                .withName("Swerve.fpvDrive");
    }

    protected static Command fpvAimDrive(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier targetRadians) {
        return swerve.applyRequest(
                        () ->
                                ROBOT_CENTRIC_FACING_ANGLE
                                        .withVelocityX(velocityX.getAsDouble())
                                        .withVelocityY(velocityY.getAsDouble())
                                        .withTargetDirection(
                                                new Rotation2d(targetRadians.getAsDouble())))
                .withName("Swerve.fpvAimDrive");
    }

    /** Drive field-relative while the CTRE heading controller holds a target angle. */
    protected static Command aimDrive(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier targetRadians) {
        return swerve.applyRequest(
                        () ->
                                FIELD_CENTRIC_FACING_ANGLE
                                        .withVelocityX(velocityX.getAsDouble())
                                        .withVelocityY(velocityY.getAsDouble())
                                        .withTargetDirection(
                                                new Rotation2d(targetRadians.getAsDouble())))
                .withName("Swerve.aimDrive");
    }

    /** Lock heading to the robot's current angle using the CTRE heading controller. */
    protected static Command headingLock(DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return Commands.defer(
                        () -> {
                            final Rotation2d locked = swerve.getRotation();
                            return swerve.applyRequest(
                                    () ->
                                            FIELD_CENTRIC_FACING_ANGLE
                                                    .withVelocityX(velocityX.getAsDouble())
                                                    .withVelocityY(velocityY.getAsDouble())
                                                    .withTargetDirection(locked));
                        },
                        Set.of(swerve))
                .withName("Swerve.HeadingLock");
    }

    protected static Command lockToClosest45deg(
            DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return Commands.defer(
                        () -> {
                            final double snapped = swerve.getClosest45();
                            return swerve.applyRequest(
                                    () ->
                                            FIELD_CENTRIC_FACING_ANGLE
                                                    .withVelocityX(velocityX.getAsDouble())
                                                    .withVelocityY(velocityY.getAsDouble())
                                                    .withTargetDirection(new Rotation2d(snapped)));
                        },
                        Set.of(swerve))
                .withName("Swerve.LockTo45deg");
    }

    protected static Command lockToClosestFieldAngle(
            DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return Commands.defer(
                        () -> {
                            final double snapped = swerve.getClosestFieldAngle();
                            return swerve.applyRequest(
                                    () ->
                                            FIELD_CENTRIC_FACING_ANGLE
                                                    .withVelocityX(velocityX.getAsDouble())
                                                    .withVelocityY(velocityY.getAsDouble())
                                                    .withTargetDirection(new Rotation2d(snapped)));
                        },
                        Set.of(swerve))
                .withName("Swerve.LockToFieldAngle");
    }

    // ------------------------------------------------------------------------
    // Swerve characterization routines
    // ------------------------------------------------------------------------
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 1; // rad/s
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.5; // rad/s^2

    public static double[] getWheelRadiusCharacterizationPositions() {
        double[] positions = new double[4];
        double wheelRadiusGuess = config.getWheelRadius().in(Meters);

        for (int i = 0; i < 4; i++) {
            positions[i] =
                    swerve.getModule(i).getCachedPosition().distanceMeters / wheelRadiusGuess;
        }
        return positions;
    }

    /**
     * Measures the robot's wheel radius by spinning in a circle (method from AdvantageKit).
     *
     * <p>This command ramps up the robot's rotation rate to a specified maximum while recording the
     * change in gyro angle and wheel positions. When the command is cancelled, it calculates and
     * prints the effective wheel radius based on the recorded data.
     */
    public static Command wheelRadiusCharacterization() {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        Commands.runOnce(() -> limiter.reset(0.0)),
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    swerve.setControl(
                                            FIELD_CENTRIC_DRIVE
                                                    .withVelocityX(0)
                                                    .withVelocityY(0)
                                                    .withRotationalRate(speed));
                                },
                                swerve)),

                // Measurement sequence
                Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(
                                () -> {
                                    state.positions = getWheelRadiusCharacterizationPositions();
                                    state.lastAngle = swerve.getRotation();
                                    state.gyroDelta = 0.0;
                                }),
                        Commands.run(
                                        () -> {
                                            var rotation = swerve.getRotation();
                                            state.gyroDelta +=
                                                    Math.abs(
                                                            rotation.minus(state.lastAngle)
                                                                    .getRadians());
                                            state.lastAngle = rotation;
                                        })
                                .finallyDo(
                                        () -> {
                                            double[] positions =
                                                    getWheelRadiusCharacterizationPositions();
                                            double wheelDelta = 0.0;
                                            for (int i = 0; i < 4; i++) {
                                                wheelDelta +=
                                                        Math.abs(positions[i] - state.positions[i])
                                                                / 4.0;
                                            }
                                            double wheelRadius =
                                                    (state.gyroDelta
                                                                    * config
                                                                            .getDrivebaseRadiusMeters())
                                                            / wheelDelta;

                                            NumberFormat formatter = new DecimalFormat("#0.000");
                                            Telemetry.log(
                                                    "WheelRadiusCharacterization/WheelDelta",
                                                    formatter.format(wheelDelta) + " radians");
                                            Telemetry.log(
                                                    "WheelRadiusCharacterization/GyroDelta",
                                                    formatter.format(state.gyroDelta) + " radians");
                                            Telemetry.log(
                                                    "WheelRadiusCharacterization/WheelRadiusMeters",
                                                    formatter.format(wheelRadius) + " meters");
                                            Telemetry.log(
                                                    "WheelRadiusCharacterization/WheelRadiusInches",
                                                    formatter.format(
                                                                    Units.metersToInches(
                                                                            wheelRadius))
                                                            + " inches");
                                        })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }

    // ------------------------------------------------------------------------
    // Reorient commands
    // ------------------------------------------------------------------------
    protected static Command reorientForward() {
        return swerve.reorientPilotAngle(0).withName("Swerve.reorientForward");
    }

    protected static Command reorientLeft() {
        return swerve.reorientPilotAngle(90).withName("Swerve.reorientLeft");
    }

    protected static Command reorientBack() {
        return swerve.reorientPilotAngle(180).withName("Swerve.reorientBack");
    }

    protected static Command reorientRight() {
        return swerve.reorientPilotAngle(270).withName("Swerve.reorientRight");
    }

    protected static Command cardinalReorient() {
        return swerve.cardinalReorient().withName("Swerve.cardinalReorient");
    }

    // ------------------------------------------------------------------------
    // Telemetry
    // ------------------------------------------------------------------------
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
