package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.reefscape.FieldConstants;
import frc.reefscape.FieldHelpers;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.Swerve;
import frc.spectrumLib.Telemetry;

public class Superstructure extends SubsystemBase {
    private final Swerve swerveSubsystem;
    private final Elevator elevatorSubsystem;

    private static final double REGULAR_TELEOP_TRANSLATION_COEFFICIENT = 1.0;

    public enum WantedSuperState {
        HOME,
        STOPPED,
        DEFAULT_STATE,

        IDLE_EMPTY,
        IDLE_ALGAE,
        IDLE_CORAL,

        ALGAE_INTAKE_FLOOR,
        ALGAE_INTAKE_L2,
        ALGAE_INTAKE_L3,

        ALGAE_NET_READY,
        ALGAE_NET_RELEASE,

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

        ALGAE_INTAKE_FLOOR,
        ALGAE_INTAKE_L2,
        ALGAE_INTAKE_L3,

        ALGAE_NET_READY,
        ALGAE_NET_RELEASE,

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

    public Superstructure(Swerve swerveSubsystem, Elevator elevatorSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    public void periodic() {
        Telemetry.log("Superstructure/WantedSuperState", wantedSuperState.toString());
        Telemetry.log("Superstructure/CurrentSuperState", currentSuperState.toString());

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
            default:
        }
    }

    private void stopped() {
        swerveSubsystem.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        elevatorSubsystem.setWantedState(Elevator.WantedState.STOPPED);
    }

    private void noPiece() {
        swerveSubsystem.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        // clawSubsystem.setWantedState(Claw.WantedState.IDLE);
        // shoulderSubsystem.setWantedState(Shoulder.WantedState.HOME);
        elevatorSubsystem.setWantedState(Elevator.WantedState.HOME);
    }

    private void holdingCoral() {
        swerveSubsystem.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        // clawSubsystem.setWantedState(Claw.WantedState.GRIP_CORAL);
        // shoulderSubsystem.setWantedState(Shoulder.WantedState.STOWED_CORAL);
        elevatorSubsystem.setWantedState(Elevator.WantedState.STOWED_CORAL);
    }

    private void holdingAlgae() {
        swerveSubsystem.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        // clawSubsystem.setWantedState(Claw.WantedState.GRIP_ALGAE);
        // shoulderSubsystem.setWantedState(Shoulder.WantedState.STOWED_ALGAE);
        elevatorSubsystem.setWantedState(Elevator.WantedState.STOWED_ALGAE);
    }

    private void scoreL4Coral(Constants.SuperstructureConstants.ScoringSide scoringSide) {

        // clawSubsystem.setWantedState(Claw.WantedState.GRIP_CORAL);
        // shoulderSubsystem.setWantedState(Shoulder.WantedState.L4_SCORE_LINEUP);
        elevatorSubsystem.setWantedState(Elevator.WantedState.L4_SCORE_LINEUP);

        driveToCoralScoringPose(scoringSide);

        if (isReadyToEject()) {
            // clawSubsystem.setWantedState(Claw.WantedState.RELEASE);
            // shoulderSubsystem.setWantedState(Shoulder.WantedState.L4_SCORE_RELEASE);
            elevatorSubsystem.setWantedState(Elevator.WantedState.L4_SCORE_RELEASE);

            //   if (!hasCoral()) {
            //         setWantedSuperState(WantedSuperState.DEFAULT_STATE);
            //   }
        }
    }

    private void scoreL3Coral(Constants.SuperstructureConstants.ScoringSide scoringSide) {

        // clawSubsystem.setWantedState(Claw.WantedState.GRIP_CORAL);
        // shoulderSubsystem.setWantedState(Shoulder.WantedState.L3_SCORE_LINEUP);
        elevatorSubsystem.setWantedState(Elevator.WantedState.L3_SCORE_LINEUP);

        driveToCoralScoringPose(scoringSide);

        if (isReadyToEject()) {
            // clawSubsystem.setWantedState(Claw.WantedState.RELEASE);
            // shoulderSubsystem.setWantedState(Shoulder.WantedState.L3_SCORE_RELEASE);
            elevatorSubsystem.setWantedState(Elevator.WantedState.L3_SCORE_RELEASE);

            //   if (!hasCoral()) {
            //         setWantedSuperState(WantedSuperState.DEFAULT_STATE);
            //   }
        }
    }

    private boolean isReadyToEject() {
        // return elevatorSubsystem.isAtSetpoint()
        //         && shoulderSubsystem.isAtSetpoint()
        //         && clawSubsystem.isAtSetpoint();
        //         swerveSubsystem.isAtDriveToPointSetpoint()
        // && swerveSubsystem.isAtDesiredRotation(Units.degreesToRadians(2.0))
        return swerveSubsystem.isAtDriveToPointSetpoint()
                && swerveSubsystem.isAtDesiredRotation(Units.degreesToRadians(2.0))
                && elevatorSubsystem.isAtSetpoint();
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

    // private boolean hasCoral() {
    //     return clawSubsystem.hasCoral();
    // }

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
                        // hasCoral();
                        () -> true),
                setStateCommand(noPieceCondition),
                // hasGamePiece();
                () -> true);
    }
}
