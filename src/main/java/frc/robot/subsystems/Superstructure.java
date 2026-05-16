package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        CORAL_L1_RELEASE,
        CORAL_L1_BACKAWAY,

        CORAL_L2_LEFT_PREP,
        CORAL_L2_LEFT_RELEASE,
        CORAL_L2_LEFT_BACKAWAY,
        CORAL_L2_RIGHT_PREP,
        CORAL_L2_RIGHT_RELEASE,
        CORAL_L2_RIGHT_BACKAWAY,

        CORAL_L3_LEFT_PREP,
        CORAL_L3_LEFT_RELEASE,
        CORAL_L3_LEFT_BACKAWAY,
        CORAL_L3_RIGHT_PREP,
        CORAL_L3_RIGHT_RELEASE,
        CORAL_L3_RIGHT_BACKAWAY,

        CORAL_L4_LEFT_PREP,
        CORAL_L4_LEFT_RELEASE,
        CORAL_L4_LEFT_BACKAWAY,
        CORAL_L4_RIGHT_PREP,
        CORAL_L4_RIGHT_RELEASE,
        CORAL_L4_RIGHT_BACKAWAY,

        CLIMING_APPROACH,
        CLIMBING_HANG,
        CLIMBING_LOCK;
    }

    private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
    private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;

    public Superstructure(Swerve swerveSubsystem, Elevator elevatorSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    public void periodic() {
        Telemetry.log("Superstructure/WantedSuperState", wantedSuperState);
        Telemetry.log("Superstructure/CurrentSuperState", currentSuperState);

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
            default:
        }
    }

    private void stopped() {
        elevatorSubsystem.setWantedState(Elevator.WantedState.STOPPED);
    }

    private void noPiece() {
        swerveSubsystem.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        elevatorSubsystem.setWantedState(Elevator.WantedState.STOPPED);
    }

    private void holdingCoral() {
        swerveSubsystem.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        elevatorSubsystem.setWantedState(Elevator.WantedState.STOWED_CORAL);
    }

    private void holdingAlgae() {
        swerveSubsystem.setWantedState(Swerve.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        elevatorSubsystem.setWantedState(Elevator.WantedState.STOWED_ALGAE);
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
                        // hasCoral();
                        () -> true),
                setStateCommand(noPieceCondition),
                // hasGamePiece();
                () -> false);
    }
}
