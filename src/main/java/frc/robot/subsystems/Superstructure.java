package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.reefscape.FieldConstants;
import frc.reefscape.FieldHelpers;
import frc.robot.constants.Constants;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.swerve.Swerve;
import frc.spectrumLib.Telemetry;

public class Superstructure extends SubsystemBase {
    private final Swerve swerveSubsystem;
    private final Elevator elevatorSubsystem;
    private final Shoulder shoulderSubsystem;
    private final Claw clawSubsystem;

    private static final double REGULAR_TELEOP_TRANSLATION_COEFFICIENT = 1.0;
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
            Claw clawSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.shoulderSubsystem = shoulderSubsystem;
        this.clawSubsystem = clawSubsystem;
    }

    public void periodic() {
        Telemetry.log("Superstructure/WantedSuperState", wantedSuperState.toString());
        Telemetry.log("Superstructure/CurrentSuperState", currentSuperState.toString());
        Telemetry.log("Superstructure/HasCoral", hasCoral());
        Telemetry.log("Superstructure/HasAlgae", hasAlgae());

        currentSuperState = handStateTransitions();
        applyStates();
    }

    private CurrentSuperState handStateTransitions() {
        switch (wantedSuperState) {
            default:
                currentSuperState = CurrentSuperState.STOPPED;
                break;
            case DEFAULT_STATE:
                // if(hasCoral()) {
                //     currentSuperState = CurrentSuperState.IDLE_CORAL;}
                // else if(hasAlgae()) {
                //     currentSuperState = CurrentSuperState.IDLE_ALGAE;}
                // else {
                //    currentSuperState = CurrentSuperState.IDLE_EMPTY;}
                currentSuperState = CurrentSuperState.IDLE_EMPTY;
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
    }

    private void noPiece() {
        swerveSubsystem.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        clawSubsystem.setWantedState(Claw.WantedState.OFF);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.HOME);
        elevatorSubsystem.setWantedState(Elevator.WantedState.HOME);
    }

    private void holdingCoral() {
        swerveSubsystem.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        clawSubsystem.setWantedState(Claw.WantedState.COLLECT_CORAL);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.STOWED_CORAL);
        elevatorSubsystem.setWantedState(Elevator.WantedState.STOWED_CORAL);
    }

    private void holdingAlgae() {
        swerveSubsystem.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        clawSubsystem.setWantedState(Claw.WantedState.COLLECT_ALGAE);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.STOWED_ALGAE);
        elevatorSubsystem.setWantedState(Elevator.WantedState.STOWED_ALGAE);
    }

    private void scoreL4Coral(Constants.SuperstructureConstants.ScoringSide scoringSide) {
        clawSubsystem.setWantedState(Claw.WantedState.COLLECT_CORAL);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.CORAL_L4_LINEUP);
        elevatorSubsystem.setWantedState(Elevator.WantedState.CORAL_L4_LINEUP);

        driveToCoralScoringPose(scoringSide);

        if (isReadyToEject()) {
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

        driveToCoralScoringPose(scoringSide);

        if (isReadyToEject()) {
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

        driveToCoralScoringPose(scoringSide);

        if (isReadyToEject()) {
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
    }

    private void intakeL3Algae(
            Constants.SuperstructureConstants.ScoringDirection scoringDirection) {
        clawSubsystem.setWantedState(Claw.WantedState.COLLECT_ALGAE);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.ALGAE_L3_INTAKE);
        elevatorSubsystem.setWantedState(Elevator.WantedState.ALGAE_L3_INTAKE);
        driveToAlgaeIntakePose(scoringDirection);
    }

    private void intakeL2Algae(
            Constants.SuperstructureConstants.ScoringDirection scoringDirection) {
        clawSubsystem.setWantedState(Claw.WantedState.COLLECT_ALGAE);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.ALGAE_L2_INTAKE);
        elevatorSubsystem.setWantedState(Elevator.WantedState.ALGAE_L2_INTAKE);
        driveToAlgaeIntakePose(scoringDirection);
    }

    private void intakeGroundAlgae() {
        clawSubsystem.setWantedState(Claw.WantedState.COLLECT_ALGAE);
        shoulderSubsystem.setWantedState(Shoulder.WantedState.ALGAE_GROUND_INTAKE);
        elevatorSubsystem.setWantedState(Elevator.WantedState.ALGAE_GROUND_INTAKE);
    }

    private boolean isReadyToEject() {
        return swerveSubsystem.isAtDriveToPointSetpoint()
                && swerveSubsystem.isAtDesiredRotation(Units.degreesToRadians(2.0))
                && elevatorSubsystem.isAtSetpoint()
                && shoulderSubsystem.isAtSetpoint();
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

    public boolean hasCoral() {
        return clawSubsystem.hasCoral();
    }

    public boolean hasAlgae() {
        return clawSubsystem.hasAlgae();
    }

    public boolean hasGamePiece() {
        return hasCoral() || hasAlgae();
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
                        () -> hasCoral()),
                setStateCommand(noPieceCondition),
                () -> hasGamePiece());
    }
}
