package frc.robot.subsystems.intake;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.RollerConfig;
import frc.spectrumLib.sim.RollerSim;
import lombok.Getter;

public class IntakeRoller extends Mechanism {

    public static class IntakeRollerConfig extends Config {
        @Getter private double hasGamePieceVelocity = 50;
        @Getter private double hasGamePieceCurrent = 80;

        /* Intake config values */
        @Getter private double currentLimit = 44;
        @Getter private double torqueCurrentLimit = 200;
        @Getter private double velocityKp = 12;
        @Getter private double velocityKv = 0.2;
        @Getter private double velocityKs = 14;

        /* Sim Configs */
        @Getter private double intakeX = (RobotSim.widthMeters / 2) + Units.inchesToMeters(10);
        @Getter private double intakeY = Units.inchesToMeters(28);
        @Getter private double wheelDiameter = 5.0;

        public IntakeRollerConfig() {
            super("IntakeRoller", 25, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(15 / 12);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
        }
    }

    // ------------------------------------------------------------------------
    // State machine
    // ------------------------------------------------------------------------
    public enum WantedState {
        INTAKE_CORAL,
        HANDOFF_CORAL,
        L1_SCORE_CORAL,
        OFF
    }

    public enum SystemState {
        INTAKE_CORAL,
        HOLD_CORAL,
        HANDOFF_CORAL,
        L1_SCORE_CORAL,
        OFF
    }

    private WantedState wantedState = WantedState.OFF;
    private SystemState systemState = SystemState.OFF;

    @Getter private IntakeRollerConfig config;
    @Getter private IntakeRollerSim sim;

    private boolean forceHoldingCoralFlag = false;

    public IntakeRoller(IntakeRollerConfig config) {
        super(config);
        this.config = config;

        simulationInit();

        Telemetry.print(getName() + " Subsystem Initialized");
    }

    // ------------------------------------------------------------------------
    // Periodic / state machine
    // ------------------------------------------------------------------------
    @Override
    public void periodic() {
        logTelemetry();

        systemState = handleStateTransition();
        applyStates();
    }

    private SystemState handleStateTransition() {
        switch (wantedState) {
            case INTAKE_CORAL:
                if (hasCoral()) {
                    return SystemState.HOLD_CORAL;
                } else {
                    return SystemState.INTAKE_CORAL;
                }
            case HANDOFF_CORAL:
                return SystemState.HANDOFF_CORAL;
            case L1_SCORE_CORAL:
                return SystemState.L1_SCORE_CORAL;
            case OFF:
                return SystemState.OFF;
            default:
                return SystemState.OFF;
        }
    }

    private void applyStates() {
        double wantedTorqueCurrent = 0.0;
        switch (systemState) {
            case INTAKE_CORAL:
                wantedTorqueCurrent = 100.0;
                break;
            case HOLD_CORAL:
                wantedTorqueCurrent = 28.0;
                break;
            case HANDOFF_CORAL:
                wantedTorqueCurrent = -50.0;
                forceHoldingCoralFlag = false;
                break;
            case L1_SCORE_CORAL:
                wantedTorqueCurrent = -30;
                forceHoldingCoralFlag = false;
                break;
            case OFF:
                stop();
                return;
            default:
                break;
        }
        final double finalWantedTorqueCurrent = wantedTorqueCurrent;
        setTorqueCurrentFoc(() -> finalWantedTorqueCurrent);
    }

    // ------------------------------------------------------------------------
    // Telemetry
    // ------------------------------------------------------------------------
    private void logTelemetry() {
        Telemetry.log("IntakeRoller/SystemState", systemState.toString());
        Telemetry.log("IntakeRoller/WantedState", wantedState.toString());
        Telemetry.log("IntakeRoller/CurrentCommand", getCurrentCommandName());
        Telemetry.log("IntakeRoller/Voltage", getVoltage(), "volts");
        Telemetry.log("IntakeRoller/StatorCurrent", getStatorCurrent(), "amps");
        Telemetry.log("IntakeRoller/SupplyCurrent", getSupplyCurrent(), "amps");
        Telemetry.log("IntakeRoller/RPM", getVelocityRPM(), "RPM");
        Telemetry.log("IntakeRoller/Temp", getTemp(), "deg_C");
        Telemetry.log("IntakeRoller/HasCoral", hasCoral());
    }

    // ------------------------------------------------------------------------
    // Public state setters
    // ------------------------------------------------------------------------
    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    // ------------------------------------------------------------------------
    // Public helpers
    // ------------------------------------------------------------------------
    public boolean hasCoral() {
        return forceHoldingCoralFlag;
    }

    public void forceSetHoldingCoralTrue() {
        this.forceHoldingCoralFlag = true;
    }

    public Command forceSetHoldingCoralTrueCommand() {
        return new InstantCommand(() -> this.forceHoldingCoralFlag = true);
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    public void simulationInit() {
        if (isAttached()) {
            sim = new IntakeRollerSim(RobotSim.frontView, motor.getSimState());
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
        }
    }

    class IntakeRollerSim extends RollerSim {
        public IntakeRollerSim(Mechanism2d mech, TalonFXSimState rollerMotorSim) {
            super(
                    new RollerConfig(config.wheelDiameter)
                            .setPosition(config.intakeX, config.intakeY)
                            .setMount(Robot.getIntakeDeploySubsystem().getSim()),
                    mech,
                    rollerMotorSim,
                    config.getName());
        }
    }
}
