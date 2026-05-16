package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.LinearConfig;
import frc.spectrumLib.sim.LinearSim;
import java.util.function.DoubleSupplier;
import lombok.*;

public class Elevator extends Mechanism {

    public static class ElevatorConfig extends Config {

        /* Elevator constants in rotations */
        @Getter @Setter private double maxRotations = 34;
        @Getter @Setter private double minRotations = 0.1;

        @Getter private double triggerTolerance = 1.15;
        @Getter private double elevatorIsUpHeight = 5;
        @Getter private double elevatorIsHighHeight = 10;
        @Getter private double initPosition = 0;
        @Getter private double holdMaxSpeedRPM = 1000;

        /* Elevator config settings */
        @Getter private final double zeroSpeed = -0.2;
        @Getter private final double positionKp = 70;
        @Getter private final double positionKd = 3.25; // 6
        @Getter private final double positionKa = 0;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 0;
        @Getter private final double positionKg = 11;
        @Getter private final double mmCruiseVelocity = 70;
        @Getter private final double mmAcceleration = 400;
        @Getter private final double mmJerk = 4500;
        @Getter private final double slowMmAcceleration = 55;
        @Getter private final double slowMmJerk = 550;

        @Getter private double currentLimit = 60; // 60;
        @Getter private double torqueCurrentLimit = 160; // 160;

        /* Sim properties */
        @Getter private double kElevatorGearing = 2.63;
        @Getter private double kCarriageMass = 13.6078;
        @Getter private double kElevatorDrumRadiusMeters = Units.inchesToMeters(0.955 / 2);
        @Getter private double initialX = RobotSim.widthMeters / 2;
        @Getter private double initialY = Units.inchesToMeters(5);
        @Getter private double angle = 90;
        @Getter private double staticLength = 40;
        @Getter private double movingLength = 40;
        @Getter public Color8Bit Stage1Color = new Color8Bit(255, 150, 0);
        @Getter public Color8Bit Stage2Color = new Color8Bit(0, 200, 255);

        public ElevatorConfig() {
            super("ElevatorFront", 40, Rio.CANIVORE);
            configMinMaxRotations(minRotations, maxRotations);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-1 * torqueCurrentLimit);
            configForwardSoftLimit(maxRotations, true);
            configReverseSoftLimit(minRotations, true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
            setFollowerConfigs(
                    new FollowerConfig(
                            "ElevatorRear", 41, Rio.CANIVORE, MotorAlignmentValue.Aligned));
        }

        /** Use these method to set the config for the mechanism on each robot */
        public void configSupplyCurrentLimit(double currentLimit) {
            this.currentLimit = currentLimit;
            configSupplyCurrentLimit(currentLimit, true);
        }
    }

    // ------------------------------------------------------------------------
    // State machine
    // ------------------------------------------------------------------------
    public enum WantedState {
        HOME,
        STOPPED,

        ALGAE_GROUND_INTAKE,
        ALGAE_INTAKE_L2,
        ALGAE_INTAKE_L3,
        ALGAE_NET,
        ALGAE_PROCESSOR,

        STOWED_CORAL,
        STOWED_ALGAE,
        PRE_CORAL_HANDOFF,
        HANDOFF,

        CORAL_L1_LINEUP,
        CORAL_L1_RELEASE,

        CORAL_L2_LINEUP,
        CORAL_L2_RELEASE,

        CORAL_L3_LINEUP,
        CORAL_L3_RELEASE,

        CORAL_L4_LINEUP,
        CORAL_L4_RELEASE,

        CLIMBING;
    }

    public enum SystemState {
        HOME,
        STOPPED,

        ALGAE_GROUND_INTAKE,
        ALGAE_INTAKE_L2,
        ALGAE_INTAKE_L3,
        ALGAE_NET,
        ALGAE_PROCESSOR,

        STOWED_CORAL,
        STOWED_ALGAE,
        PRE_CORAL_HANDOFF,
        HANDOFF,

        CORAL_L1_LINEUP,
        CORAL_L1_RELEASE,

        CORAL_L2_LINEUP,
        CORAL_L2_RELEASE,

        CORAL_L3_LINEUP,
        CORAL_L3_RELEASE,

        CORAL_L4_LINEUP,
        CORAL_L4_RELEASE,

        CLIMBING;
    }

    private WantedState wantedState = WantedState.STOPPED;
    private SystemState systemState = SystemState.STOPPED;

    @Getter private ElevatorConfig config;
    @Getter private ElevatorSim sim;

    public Elevator(ElevatorConfig config) {
        super(config);
        this.config = config;

        setInitialPosition();

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
            case HOME:
                return SystemState.HOME;
            case STOPPED:
                return SystemState.STOPPED;
            case ALGAE_GROUND_INTAKE:
                return SystemState.ALGAE_GROUND_INTAKE;
            case ALGAE_INTAKE_L2:
                return SystemState.ALGAE_INTAKE_L2;
            case ALGAE_INTAKE_L3:
                return SystemState.ALGAE_INTAKE_L3;
            case ALGAE_NET:
                return SystemState.ALGAE_NET;
            case ALGAE_PROCESSOR:
                return SystemState.ALGAE_PROCESSOR;
            case STOWED_CORAL:
                return SystemState.STOWED_CORAL;
            case STOWED_ALGAE:
                return SystemState.STOWED_ALGAE;
            case PRE_CORAL_HANDOFF:
                return SystemState.PRE_CORAL_HANDOFF;
            case HANDOFF:
                return SystemState.HANDOFF;
            case CORAL_L1_LINEUP:
                return SystemState.CORAL_L1_LINEUP;
            case CORAL_L1_RELEASE:
                return SystemState.CORAL_L1_RELEASE;
            case CORAL_L2_LINEUP:
                return SystemState.CORAL_L2_LINEUP;
            case CORAL_L2_RELEASE:
                return SystemState.CORAL_L2_RELEASE;
            case CORAL_L3_LINEUP:
                return SystemState.CORAL_L3_LINEUP;
            case CORAL_L3_RELEASE:
                return SystemState.CORAL_L3_RELEASE;
            case CORAL_L4_LINEUP:
                return SystemState.CORAL_L4_LINEUP;
            case CORAL_L4_RELEASE:
                return SystemState.CORAL_L4_RELEASE;
            case CLIMBING:
                return SystemState.CLIMBING;
            default:
                return systemState;
        }
    }

    private void applyStates() {
        double wantedPosition = 0.0;
        switch (systemState) {
            case HOME:
                wantedPosition = 0.5;
                break;
            case STOPPED:
                stop();
                break;
            case ALGAE_GROUND_INTAKE:
                wantedPosition = 10.5;
                break;
            case ALGAE_INTAKE_L2:
                wantedPosition = 14.4;
                break;
            case ALGAE_INTAKE_L3:
                wantedPosition = 24.9;
                break;
            case ALGAE_NET:
                wantedPosition = 33.966;
                break;
            case ALGAE_PROCESSOR:
                wantedPosition = 0.0;
                break;
            case STOWED_CORAL:
                wantedPosition = 0.5;
                break;
            case STOWED_ALGAE:
                wantedPosition = 0.5;
                break;
            case PRE_CORAL_HANDOFF:
                wantedPosition = 31.0;
                break;
            case HANDOFF:
                wantedPosition = 27.4;
                break;
            case CORAL_L1_LINEUP:
                wantedPosition = 1.5;
                break;
            case CORAL_L1_RELEASE:
                wantedPosition = 0.5;
                break;
            case CORAL_L2_LINEUP:
                wantedPosition = 7.3;
                break;
            case CORAL_L2_RELEASE:
                wantedPosition = 7.3;
                break;
            case CORAL_L3_LINEUP:
                wantedPosition = 17.2;
                break;
            case CORAL_L3_RELEASE:
                wantedPosition = 17.2;
                break;
            case CORAL_L4_LINEUP:
                wantedPosition = 33.0;
                break;
            case CORAL_L4_RELEASE:
                wantedPosition = 33.0;
                break;
            case CLIMBING:
                wantedPosition = 0.5;
                break;
            default:
                break;
        }
        final double finalWantedPosition = wantedPosition;
        setMMPositionFoc(() -> finalWantedPosition);
    }

    private void logTelemetry() {
        Telemetry.log("Elevator/SystemState", systemState.toString());
        Telemetry.log("Elevator/WantedState", wantedState.toString());
        Telemetry.log("Elevator/Voltage", getVoltage(), "volts");
        Telemetry.log("Elevator/StatorCurrent", getStatorCurrent(), "amps");
        Telemetry.log("Elevator/SupplyCurrent", getSupplyCurrent(), "amps");
        Telemetry.log("Elevator/Temp", getTemp(), "deg_C");
        Telemetry.log("Elevator/PositionRotations", getPositionRotations(), "rotations");
        Telemetry.log("Elevator/PositionMeters", getPositionMeters.getAsDouble(), "meters");
    }

    // ------------------------------------------------------------------------
    // Public state setters
    // ------------------------------------------------------------------------
    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    // ------------------------------------------------------------------------
    // Helper methods
    // ------------------------------------------------------------------------
    private void setInitialPosition() {
        if (isAttached()) {
            motor.setPosition(config.getInitPosition());
            followerMotors[0].setPosition(config.getInitPosition());
        }
    }

    public Command resetToInitialPos() {
        return run(this::setInitialPosition);
    }

    public boolean isAtSetpoint() {
        return isAtTargetPosition(config::getTriggerTolerance);
    }

    public DoubleSupplier getPositionMeters =
            () -> {
                if (isAttached()) {
                    return getPositionRotations()
                            * config.getKElevatorGearing()
                            * (2 * Math.PI * config.getKElevatorDrumRadiusMeters());
                } else {
                    return 0.0;
                }
            };

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            sim = new ElevatorSim(motor.getSimState(), RobotSim.leftView);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
        }
    }

    class ElevatorSim extends LinearSim {
        public ElevatorSim(TalonFXSimState elevatorMotorSim, Mechanism2d mech) {
            super(
                    new LinearConfig(
                                    config.initialX,
                                    config.initialY,
                                    config.kElevatorGearing,
                                    config.kElevatorDrumRadiusMeters)
                            .setAngle(config.angle)
                            .setMovingLength(config.getMovingLength())
                            .setStaticLength(config.getStaticLength())
                            .setMaxHeight(40),
                    mech,
                    elevatorMotorSim,
                    "1" + config.getName()); // added 1 to the name to create it first
        }
    }
}
