package frc.robot.subsystems.claw;

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

public class Claw extends Mechanism {

    public static class ClawConfig extends Config {
        @Getter private double hasGamePieceVelocity = 50;
        @Getter private double hasGamePieceCurrent = 80;
        @Getter private double hasAlgaeCurrent = 80;
        @Getter private double scoreDelay = 0.2;

        /* Intake config values */
        @Getter private double currentLimit = 44;
        @Getter private double torqueCurrentLimit = 200;
        @Getter private double velocityKp = 12;
        @Getter private double velocityKv = 0.2;
        @Getter private double velocityKs = 14;

        /* Sim Configs */
        @Getter private double intakeX = RobotSim.widthMeters / 2;
        @Getter private double intakeY = Units.inchesToMeters(42);
        @Getter private double wheelDiameter = 5.0;

        public ClawConfig() {
            super("Claw", 5, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(12);
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
        COLLECT_ALGAE,
        EJECT_ALGAE,
        EJECT_ALGAE_PROCESSOR,

        COLLECT_CORAL,
        EJECT_CORAL,

        OFF
    }

    public enum SystemState {
        COLLECT_ALGAE,
        HOLD_ALGAE,
        EJECT_ALGAE,
        EJECT_ALGAE_PROCESSOR,

        COLLECT_CORAL,
        HOLD_CORAL,
        EJECT_CORAL,

        OFF
    }

    private WantedState wantedState = WantedState.OFF;
    private SystemState systemState = SystemState.OFF;

    @Getter private ClawConfig config;
    @Getter private ClawSim sim;

    private boolean forceHoldingCoralFlag = false;
    private boolean forceHoldingAlgaeFlag = false;

    public Claw(ClawConfig config) {
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
            case COLLECT_ALGAE:
                if (hasAlgae()) {
                    return SystemState.HOLD_ALGAE;
                } else {
                    return SystemState.COLLECT_ALGAE;
                }
            case EJECT_ALGAE:
                return SystemState.EJECT_ALGAE;
            case EJECT_ALGAE_PROCESSOR:
                return SystemState.EJECT_ALGAE_PROCESSOR;
            case COLLECT_CORAL:
                if (hasCoral()) {
                    return SystemState.HOLD_CORAL;
                } else {
                    return SystemState.COLLECT_CORAL;
                }
            case EJECT_CORAL:
                return SystemState.EJECT_CORAL;
            case OFF:
                return SystemState.OFF;
            default:
                return SystemState.OFF;
        }
    }

    private void applyStates() {
        double wantedTorqueCurrent = 0.0;
        switch (systemState) {
            case COLLECT_ALGAE:
                wantedTorqueCurrent = 150.0;
                break;
            case HOLD_ALGAE:
                wantedTorqueCurrent = 150.0;
                break;
            case EJECT_ALGAE:
                wantedTorqueCurrent = -200.0;
                forceHoldingAlgaeFlag = false;
                break;
            case EJECT_ALGAE_PROCESSOR:
                wantedTorqueCurrent = -50.0;
                forceHoldingAlgaeFlag = false;
                break;
            case COLLECT_CORAL:
                wantedTorqueCurrent = 30.0;
                break;
            case HOLD_CORAL:
                wantedTorqueCurrent = 5.0;
                break;
            case EJECT_CORAL:
                wantedTorqueCurrent = -25.0;
                forceHoldingCoralFlag = false;
                break;
            case OFF:
                stop();
                break;
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
        Telemetry.log("Claw/SystemState", systemState.toString());
        Telemetry.log("Claw/WantedState", wantedState.toString());
        Telemetry.log("Claw/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Claw/Voltage", getVoltage(), "volts");
        Telemetry.log("Claw/StatorCurrent", getStatorCurrent(), "amps");
        Telemetry.log("Claw/SupplyCurrent", getSupplyCurrent(), "amps");
        Telemetry.log("Claw/RPM", getVelocityRPM(), "RPM");
        Telemetry.log("Claw/Temp", getTemp(), "deg_C");
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

    public boolean hasAlgae() {
        return forceHoldingAlgaeFlag;
    }

    public void forceSetHoldingCoralTrue() {
        this.forceHoldingCoralFlag = true;
    }

    public Command forceSetHoldingCoralTrueCommand() {
        return new InstantCommand(() -> this.forceHoldingCoralFlag = true);
    }

    public void forceSetHoldingAlgaeTrue() {
        this.forceHoldingAlgaeFlag = true;
    }

    public Command forceSetHoldingAlgaeTrueCommand() {
        return new InstantCommand(() -> this.forceHoldingAlgaeFlag = true);
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    public void simulationInit() {
        if (isAttached()) {
            sim = new ClawSim(RobotSim.leftView, motor.getSimState());
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
        }
    }

    class ClawSim extends RollerSim {
        public ClawSim(Mechanism2d mech, TalonFXSimState coralRollerMotorSim) {
            super(
                    new RollerConfig(config.wheelDiameter)
                            .setPosition(config.intakeX, config.intakeY)
                            .setMount(Robot.getShoulderSubsystem().getSim()),
                    mech,
                    coralRollerMotorSim,
                    config.getName());
        }
    }
}
