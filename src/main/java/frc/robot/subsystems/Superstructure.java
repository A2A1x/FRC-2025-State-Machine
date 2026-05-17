package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.reefscape.FieldConstants;
import frc.reefscape.FieldHelpers;
import frc.robot.constants.Constants;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.IntakeDeploy;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.swerve.Swerve;
import frc.spectrumLib.Telemetry;

public class Superstructure extends SubsystemBase {
    private final Swerve swerveSubsystem;
    private final Elevator elevatorSubsystem;
    private final Shoulder shoulderSubsystem;
    private final Claw clawSubsystem;
    private final IntakeRoller intakeRollerSubsystem;
    private final IntakeDeploy intakeDeploySubsystem;

    private Trigger readyForClawEject;
    private Trigger readyForIntakeEject;
    private Trigger clawHasCoral;
    private Trigger clawHasAlgae;
    private Trigger clawHasGamePiece;
    private Trigger intakeHasCoral;

    private static final double REGULAR_TELEOP_TRANSLATION_COEFFICIENT = 1.0;
    private static final double INTAKE_GROUND_CORAL_TELEOP_TRANSLATION_COEFFICIENT = 0.4;
    private static final double ARM_HIGH_TELEOP_TRANSLATION_COEFFICIENT = 0.1;

    public enum WantedSuperState {
        HOME,
        STOPPED,
        DEFAULT_STATE,

        IDLE_EMPTY,
        IDLE_ALGAE,
        IDLE_CORAL,

        ALGAE_GROUND_INTAKE,
        ALGAE_L2_INTAKE,
        ALGAE_L3_INTAKE,
        ALGAE_NET_PREP,
        ALGAE_NET_SCORE,
        ALGAE_PROCESSOR_SCORE,

        CORAL_INTAKE_FLOOR,

        CORAL_HANDOFF,

        CORAL_L1_PREP,
        CORAL_L1_SCORE,

        CORAL_L2_LEFT_SCORE,
        CORAL_L2_RIGHT_SCORE,

        CORAL_L3_LEFT_SCORE,
        CORAL_L3_RIGHT_SCORE,

        CORAL_L4_LEFT_SCORE,
        CORAL_L4_RIGHT_SCORE,

        CLIMING_APPROACH,
        CLIMBING_HANG,
        CLIMBING_LOCK;
    }

    public enum CurrentSuperState {
        HOME,
        STOPPED,
        DEFAULT_STATE,

        IDLE_EMPTY,
        IDLE_ALGAE,
        IDLE_CORAL,

        ALGAE_GROUND_INTAKE,
        ALGAE_L2_INTAKE,
        ALGAE_L3_INTAKE,
        ALGAE_NET_PREP,
        ALGAE_NET_SCORE,
        ALGAE_PROCESSOR_SCORE,

        CORAL_INTAKE_FLOOR,

        CORAL_PREPARE_HANDOFF,
        CORAL_RELEASE_HANDOFF,

        CORAL_L1_PREP,
        CORAL_L1_SCORE,

        CORAL_L2_LEFT_SCORE,
        CORAL_L2_RIGHT_SCORE,

        CORAL_L3_LEFT_SCORE,
        CORAL_L3_RIGHT_SCORE,

        CORAL_L4_LEFT_SCORE,
        CORAL_L4_RIGHT_SCORE,

        CLIMING_APPROACH,
        CLIMBING_HANG,
        CLIMBING_LOCK;
    }

    private WantedSuperState wantedSuperState = WantedSuperState.DEFAULT_STATE;
    private CurrentSuperState currentSuperState = CurrentSuperState.DEFAULT_STATE;

    public Superstructure(
            Swerve swerveSubsystem,
            Elevator elevatorSubsystem,
            Shoulder shoulderSubsystem,
            Claw clawSubsystem,
            IntakeRoller intakeRollerSubsystem,
            IntakeDeploy intakeDeploySubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.shoulderSubsystem = shoulderSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.intakeRollerSubsystem = intakeRollerSubsystem;
        this.intakeDeploySubsystem = intakeDeploySubsystem;

        setupTriggers();
    }

    private void setupTriggers() {
        readyForClawEject =
                new Trigger(
                                () ->
                                        swerveSubsystem.isAtDriveToPointSetpoint()
                                                && swerveSubsystem.isAtDesiredRotation(
                                                        Units.degreesToRadians(2.0))
                                                && elevatorSubsystem.isAtSetpoint()
                                                && shoulderSubsystem.isAtSetpoint())
                        .debounce(
                                Constants.SuperstructureConstants
                                        .TELEOP_MECHANISM_AT_SETPOINT_DEBOUNCE_SEC);

        readyForIntakeEject =
                new Trigger(
                                () ->
                                        elevatorSubsystem.isAtSetpoint()
                                                && shoulderSubsystem.isAtSetpoint()
                                                && intakeDeploySubsystem.isAtHandoffSetpoint())
                        .debounce(
                                Constants.SuperstructureConstants
                                        .TELEOP_MECHANISM_AT_SETPOINT_DEBOUNCE_SEC);

        clawHasCoral =
                new Trigger(() -> clawSubsystem.hasCoral())
                        .debounce(
                                Constants.SuperstructureConstants
                                        .TELEOP_MECHANISM_HAS_GAME_PIECE_DEBOUNCE_SEC);

        clawHasAlgae =
                new Trigger(() -> clawSubsystem.hasAlgae())
                        .debounce(
                                Constants.SuperstructureConstants
                                        .TELEOP_MECHANISM_HAS_GAME_PIECE_DEBOUNCE_SEC);

        clawHasGamePiece =
                new Trigger(() -> clawSubsystem.hasCoral() || clawSubsystem.hasAlgae())
                        .debounce(
                                Constants.SuperstructureConstants
                                        .TELEOP_MECHANISM_HAS_GAME_PIECE_DEBOUNCE_SEC);

        intakeHasCoral =
                new Trigger(() -> intakeRollerSubsystem.hasCoral())
                        .debounce(
                                Constants.SuperstructureConstants
                                        .TELEOP_MECHANISM_HAS_GAME_PIECE_DEBOUNCE_SEC);
    }

    public void periodic() {
        Telemetry.log("Superstructure/WantedSuperState", wantedSuperState.toString());
        Telemetry.log("Superstructure/CurrentSuperState", currentSuperState.toString());
        Telemetry.log("Superstructure/ClawHasCoral", clawHasCoral.getAsBoolean());
        Telemetry.log("Superstructure/ClawHasAlgae", clawHasAlgae.getAsBoolean());
        Telemetry.log("Superstructure/IntakeHasCoral", intakeHasCoral.getAsBoolean());
        Telemetry.log("Superstructure/ReadyForClawEject", readyForClawEject.getAsBoolean());
        Telemetry.log("Superstructure/ReadyForIntakeEject", readyForIntakeEject.getAsBoolean());

        currentSuperState = handStateTransitions();
        applyStates();
    }

    private CurrentSuperState handStateTransitions() {
        switch (wantedSuperState) {
            default:
                currentSuperState = CurrentSuperState.STOPPED;
                break;
            case DEFAULT_STATE:
                if (clawHasGamePiece.getAsBoolean()) {
                    if (clawHasCoral.getAsBoolean()) {
                        currentSuperState = CurrentSuperState.IDLE_CORAL;
                    } else if (clawHasAlgae.getAsBoolean()) {
                        currentSuperState = CurrentSuperState.IDLE_ALGAE;
                    }
                } else if (currentSuperState == CurrentSuperState.CORAL_INTAKE_FLOOR
                        && intakeHasCoral.getAsBoolean()) {
                    wantedSuperState = WantedSuperState.CORAL_HANDOFF;
                } else {
                    wantedSuperState = WantedSuperState.IDLE_EMPTY;
                }
                break;
            case HOME:
                currentSuperState = CurrentSuperState.HOME;
                break;
            case IDLE_EMPTY:
                currentSuperState = CurrentSuperState.IDLE_EMPTY;
                break;
            case IDLE_ALGAE:
                currentSuperState = CurrentSuperState.IDLE_ALGAE;
                break;
            case IDLE_CORAL:
                currentSuperState = CurrentSuperState.IDLE_CORAL;
                break;
            case CORAL_INTAKE_FLOOR:
                currentSuperState = CurrentSuperState.CORAL_INTAKE_FLOOR;
                break;
            case CORAL_HANDOFF:
                if (clawHasCoral.getAsBoolean()) {
                    wantedSuperState = WantedSuperState.IDLE_CORAL;
                } else if (readyForIntakeEject.getAsBoolean()
                        || currentSuperState == CurrentSuperState.CORAL_RELEASE_HANDOFF) {
                    currentSuperState = CurrentSuperState.CORAL_RELEASE_HANDOFF;
                } else {
                    currentSuperState = CurrentSuperState.CORAL_PREPARE_HANDOFF;
                }
                break;
            case CORAL_L4_LEFT_SCORE:
                currentSuperState = CurrentSuperState.CORAL_L4_LEFT_SCORE;
                break;
            case CORAL_L4_RIGHT_SCORE:
                currentSuperState = CurrentSuperState.CORAL_L4_RIGHT_SCORE;
                break;
            case CORAL_L3_LEFT_SCORE:
                currentSuperState = CurrentSuperState.CORAL_L3_LEFT_SCORE;
                break;
            case CORAL_L3_RIGHT_SCORE:
                currentSuperState = CurrentSuperState.CORAL_L3_RIGHT_SCORE;
                break;
            case CORAL_L2_LEFT_SCORE:
                currentSuperState = CurrentSuperState.CORAL_L2_LEFT_SCORE;
                break;
            case CORAL_L2_RIGHT_SCORE:
                currentSuperState = CurrentSuperState.CORAL_L2_RIGHT_SCORE;
                break;
            case ALGAE_GROUND_INTAKE:
                currentSuperState = CurrentSuperState.ALGAE_GROUND_INTAKE;
                break;
            case ALGAE_L2_INTAKE:
                currentSuperState = CurrentSuperState.ALGAE_L2_INTAKE;
                break;
            case ALGAE_L3_INTAKE:
                currentSuperState = CurrentSuperState.ALGAE_L3_INTAKE;
                break;
            case ALGAE_NET_PREP:
                currentSuperState = CurrentSuperState.ALGAE_NET_PREP;
                break;
            case ALGAE_NET_SCORE:
                currentSuperState = CurrentSuperState.ALGAE_NET_SCORE;
                break;
        }
        return currentSuperState;
    }

    private void applyStates() {
        switch (currentSuperState) {
            case STOPPED:
                stopped();
                break;
            case IDLE_EMPTY:
                noPiece();
                break;
            case IDLE_CORAL:
                holdingCoral();
                break;
            case IDLE_ALGAE:
                holdingAlgae();
                break;
            case CORAL_INTAKE_FLOOR:
                intakeGroundCoral();
                break;
            case CORAL_PREPARE_HANDOFF:
                prepareCoralHandoff();
                break;
            case CORAL_RELEASE_HANDOFF:
                executeCoralHandoff();
                break;
            case CORAL_L4_LEFT_SCORE:
                scoreL4Coral(Constants.SuperstructureConstants.ScoringSide.LEFT);
                break;
            case CORAL_L4_RIGHT_SCORE:
                scoreL4Coral(Constants.SuperstructureConstants.ScoringSide.RIGHT);
                break;
            case CORAL_L3_LEFT_SCORE:
                scoreL3Coral(Constants.SuperstructureConstants.ScoringSide.LEFT);
                break;
            case CORAL_L3_RIGHT_SCORE:
                scoreL3Coral(Constants.SuperstructureConstants.ScoringSide.RIGHT);
                break;
            case CORAL_L2_LEFT_SCORE:
                scoreL2Coral(Constants.SuperstructureConstants.ScoringSide.LEFT);
                break;
            case CORAL_L2_RIGHT_SCORE:
                scoreL2Coral(Constants.SuperstructureConstants.ScoringSide.RIGHT);
                break;
            case ALGAE_GROUND_INTAKE:
                intakeGroundAlgae();
                break;
            case ALGAE_L2_INTAKE:
                intakeL2Algae(Constants.SuperstructureConstants.ScoringDirection.FRONT);
                break;
            case ALGAE_L3_INTAKE:
                intakeL3Algae(Constants.SuperstructureConstants.ScoringDirection.FRONT);
                break;
            case ALGAE_NET_PREP:
                prepAlgaeNet();
                break;
            case ALGAE_NET_SCORE:
                scoreAlgaeNet();
                break;
            default:
        }
    }

    private void stopped() {
        swerveSubsystem.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        elevatorSubsystem.setWantedState(Elevator.WantedState.STOPPED);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.STOPPED);
        clawSubsystem.setWantedState(Claw.WantedState.OFF);
        intakeRollerSubsystem.setWantedState(IntakeRoller.WantedState.OFF);
        intakeDeploySubsystem.setWantedState(IntakeDeploy.WantedState.STOPPED);
    }

    private void noPiece() {
        swerveSubsystem.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        clawSubsystem.setWantedState(Claw.WantedState.OFF);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.HOME);
        elevatorSubsystem.setWantedState(Elevator.WantedState.HOME);
        intakeRollerSubsystem.setWantedState(IntakeRoller.WantedState.OFF);
        intakeDeploySubsystem.setWantedState(IntakeDeploy.WantedState.HOME);
    }

    private void holdingCoral() {
        swerveSubsystem.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        clawSubsystem.setWantedState(Claw.WantedState.COLLECT_CORAL);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.STOWED_CORAL);
        elevatorSubsystem.setWantedState(Elevator.WantedState.STOWED_CORAL);
        intakeRollerSubsystem.setWantedState(IntakeRoller.WantedState.OFF);
        intakeDeploySubsystem.setWantedState(IntakeDeploy.WantedState.HOME);
    }

    private void holdingAlgae() {
        swerveSubsystem.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        clawSubsystem.setWantedState(Claw.WantedState.COLLECT_ALGAE);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.STOWED_ALGAE);
        elevatorSubsystem.setWantedState(Elevator.WantedState.STOWED_ALGAE);
        intakeRollerSubsystem.setWantedState(IntakeRoller.WantedState.OFF);
        intakeDeploySubsystem.setWantedState(IntakeDeploy.WantedState.HOME);
    }

    private void intakeGroundCoral() {
        swerveSubsystem.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(
                INTAKE_GROUND_CORAL_TELEOP_TRANSLATION_COEFFICIENT);
        clawSubsystem.setWantedState(Claw.WantedState.OFF);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.PRE_CORAL_HANDOFF);
        elevatorSubsystem.setWantedState(Elevator.WantedState.PRE_CORAL_HANDOFF);
        intakeRollerSubsystem.setWantedState(IntakeRoller.WantedState.INTAKE_CORAL);
        intakeDeploySubsystem.setWantedState(IntakeDeploy.WantedState.GROUND_CORAL_INTAKE);
    }

    private void prepareCoralHandoff() {
        swerveSubsystem.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        clawSubsystem.setWantedState(Claw.WantedState.COLLECT_CORAL);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.PRE_CORAL_HANDOFF);
        elevatorSubsystem.setWantedState(Elevator.WantedState.PRE_CORAL_HANDOFF);
        intakeRollerSubsystem.setWantedState(IntakeRoller.WantedState.INTAKE_CORAL);
        intakeDeploySubsystem.setWantedState(IntakeDeploy.WantedState.HANDOFF);
    }

    private void executeCoralHandoff() {
        swerveSubsystem.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        clawSubsystem.setWantedState(Claw.WantedState.COLLECT_CORAL);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.HANDOFF);
        elevatorSubsystem.setWantedState(Elevator.WantedState.HANDOFF);
        intakeRollerSubsystem.setWantedState(IntakeRoller.WantedState.HANDOFF_CORAL);
        intakeDeploySubsystem.setWantedState(IntakeDeploy.WantedState.HANDOFF);
    }

    private void scoreL4Coral(Constants.SuperstructureConstants.ScoringSide scoringSide) {
        clawSubsystem.setWantedState(Claw.WantedState.COLLECT_CORAL);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.CORAL_L4_LINEUP);
        elevatorSubsystem.setWantedState(Elevator.WantedState.CORAL_L4_LINEUP);
        intakeRollerSubsystem.setWantedState(IntakeRoller.WantedState.OFF);
        intakeDeploySubsystem.setWantedState(IntakeDeploy.WantedState.HOME);

        driveToCoralScoringPose(scoringSide);

        if (readyForClawEject.getAsBoolean()) {
            clawSubsystem.setWantedState(Claw.WantedState.EJECT_CORAL);
            shoulderSubsystem.setWantedState(Shoulder.WantedState.CORAL_L4_RELEASE);
            elevatorSubsystem.setWantedState(Elevator.WantedState.CORAL_L4_RELEASE);

            //   if (!hasCoral()) {
            //         setWantedSuperState(WantedSuperState.DEFAULT_STATE);
            //   }
        }
    }

    private void scoreL3Coral(Constants.SuperstructureConstants.ScoringSide scoringSide) {
        clawSubsystem.setWantedState(Claw.WantedState.COLLECT_CORAL);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.CORAL_L3_LINEUP);
        elevatorSubsystem.setWantedState(Elevator.WantedState.CORAL_L3_LINEUP);
        intakeRollerSubsystem.setWantedState(IntakeRoller.WantedState.OFF);
        intakeDeploySubsystem.setWantedState(IntakeDeploy.WantedState.HOME);

        driveToCoralScoringPose(scoringSide);

        if (readyForClawEject.getAsBoolean()) {
            clawSubsystem.setWantedState(Claw.WantedState.EJECT_CORAL);
            shoulderSubsystem.setWantedState(Shoulder.WantedState.CORAL_L3_RELEASE);
            elevatorSubsystem.setWantedState(Elevator.WantedState.CORAL_L3_RELEASE);

            //   if (!hasCoral()) {
            //         setWantedSuperState(WantedSuperState.DEFAULT_STATE);
            //   }
        }
    }

    private void scoreL2Coral(Constants.SuperstructureConstants.ScoringSide scoringSide) {
        clawSubsystem.setWantedState(Claw.WantedState.COLLECT_CORAL);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.CORAL_L2_LINEUP);
        elevatorSubsystem.setWantedState(Elevator.WantedState.CORAL_L2_LINEUP);
        intakeRollerSubsystem.setWantedState(IntakeRoller.WantedState.OFF);
        intakeDeploySubsystem.setWantedState(IntakeDeploy.WantedState.HOME);

        driveToCoralScoringPose(scoringSide);

        if (readyForClawEject.getAsBoolean()) {
            clawSubsystem.setWantedState(Claw.WantedState.EJECT_CORAL);
            shoulderSubsystem.setWantedState(Shoulder.WantedState.CORAL_L2_RELEASE);
            elevatorSubsystem.setWantedState(Elevator.WantedState.CORAL_L2_RELEASE);

            //   if (!hasCoral()) {
            //         setWantedSuperState(WantedSuperState.DEFAULT_STATE);
            //   }
        }
    }

    private void prepAlgaeNet() {
        swerveSubsystem.setTargetRotation(Rotation2d.kZero);
        swerveSubsystem.setTeleopVelocityCoefficient(ARM_HIGH_TELEOP_TRANSLATION_COEFFICIENT);
        intakeRollerSubsystem.setWantedState(IntakeRoller.WantedState.OFF);
        intakeDeploySubsystem.setWantedState(IntakeDeploy.WantedState.HOME);

        if (swerveSubsystem.isAtDesiredRotation()) {
            clawSubsystem.setWantedState(Claw.WantedState.COLLECT_ALGAE);
            shoulderSubsystem.setWantedState(Shoulder.WantedState.ALGAE_NET);
            elevatorSubsystem.setWantedState(Elevator.WantedState.ALGAE_NET);
        }
    }

    private void scoreAlgaeNet() {
        swerveSubsystem.setTargetRotation(Rotation2d.kZero);
        swerveSubsystem.setTeleopVelocityCoefficient(ARM_HIGH_TELEOP_TRANSLATION_COEFFICIENT);
        clawSubsystem.setWantedState(Claw.WantedState.EJECT_ALGAE);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.ALGAE_NET);
        elevatorSubsystem.setWantedState(Elevator.WantedState.ALGAE_NET);
        intakeRollerSubsystem.setWantedState(IntakeRoller.WantedState.OFF);
        intakeDeploySubsystem.setWantedState(IntakeDeploy.WantedState.HOME);
    }

    private void intakeL3Algae(
            Constants.SuperstructureConstants.ScoringDirection scoringDirection) {
        clawSubsystem.setWantedState(Claw.WantedState.COLLECT_ALGAE);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.ALGAE_L3_INTAKE);
        elevatorSubsystem.setWantedState(Elevator.WantedState.ALGAE_L3_INTAKE);
        intakeRollerSubsystem.setWantedState(IntakeRoller.WantedState.OFF);
        intakeDeploySubsystem.setWantedState(IntakeDeploy.WantedState.HOME);
        driveToAlgaeIntakePose(scoringDirection);
    }

    private void intakeL2Algae(
            Constants.SuperstructureConstants.ScoringDirection scoringDirection) {
        clawSubsystem.setWantedState(Claw.WantedState.COLLECT_ALGAE);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.ALGAE_L2_INTAKE);
        elevatorSubsystem.setWantedState(Elevator.WantedState.ALGAE_L2_INTAKE);
        intakeRollerSubsystem.setWantedState(IntakeRoller.WantedState.OFF);
        intakeDeploySubsystem.setWantedState(IntakeDeploy.WantedState.HOME);
        driveToAlgaeIntakePose(scoringDirection);
    }

    private void intakeGroundAlgae() {
        clawSubsystem.setWantedState(Claw.WantedState.COLLECT_ALGAE);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.ALGAE_GROUND_INTAKE);
        elevatorSubsystem.setWantedState(Elevator.WantedState.ALGAE_GROUND_INTAKE);
        intakeRollerSubsystem.setWantedState(IntakeRoller.WantedState.OFF);
        intakeDeploySubsystem.setWantedState(IntakeDeploy.WantedState.HOME);
    }

    private void driveToCoralScoringPose(
            Constants.SuperstructureConstants.ScoringSide scoringSide) {
        Pose2d desiredPoseToDriveTo =
                FieldConstants.getDesiredFinalScoringPoseForCoral(
                        FieldHelpers.getReefZoneTagID(swerveSubsystem.getRobotPose()),
                        scoringSide,
                        Constants.SuperstructureConstants.ScoringDirection.FRONT);
        swerveSubsystem.setDesiredPoseForDriveToPoint(desiredPoseToDriveTo);
    }

    private void driveToAlgaeIntakePose(
            Constants.SuperstructureConstants.ScoringDirection scoringDirection) {
        Pose2d desiredPoseToDriveTo =
                FieldConstants.getDesiredPointToDriveToForAlgaeIntaking(
                        FieldHelpers.getReefZoneTagID(swerveSubsystem.getRobotPose()),
                        scoringDirection);
        swerveSubsystem.setDesiredPoseForDriveToPoint(desiredPoseToDriveTo);
    }

    public boolean armLow() {
        return shoulderSubsystem.shoulderLow();
    }

    public boolean elevatorLow() {
        return elevatorSubsystem.elevatorLow();
    }

    public void setWantedSuperState(WantedSuperState superState) {
        this.wantedSuperState = superState;
    }

    public Command setStateCommand(WantedSuperState superState) {
        return new InstantCommand(() -> setWantedSuperState(superState));
    }

    public Command configureButtonBinding(
            WantedSuperState hasCoralCondition,
            WantedSuperState hasAlgaeCondition,
            WantedSuperState noPieceCondition) {
        return Commands.either(
                Commands.either(
                        setStateCommand(hasCoralCondition),
                        setStateCommand(hasAlgaeCondition),
                        () -> clawHasCoral.getAsBoolean()),
                setStateCommand(noPieceCondition),
                () -> clawHasGamePiece.getAsBoolean());
    }
}
