package frc.robot.subsystems.intake;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

public class IntakeDeploy extends Mechanism {

    public static class IntakeDeployConfig extends Config {
        @Getter @Setter private double toleranceRotations = Units.degreesToRotations(3);

        @Getter @Setter private double offset = 90;
        @Getter @Setter private double initPosition = 0;

        /* IntakePivot config settings */
        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18;

        @Getter private final double currentLimit = 60;
        @Getter private final double torqueCurrentLimit = 180;
        @Getter private final double positionKp = 190;
        @Getter private final double positionKd = 40;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 0.3;
        @Getter private final double positionKa = 0.001;
        @Getter private final double positionKg = 2.9;
        @Getter private final double mmCruiseVelocity = 1;
        @Getter private final double mmAcceleration = 10;
        @Getter private final double mmJerk = 0;

        @Getter @Setter private double sensorToMechanismRatio = 99.5555555555;
        @Getter @Setter private double rotorToSensorRatio = 1;

        /* Sim properties */
        @Getter
        private double intakeDeployX = (RobotSim.widthMeters / 2) + Units.inchesToMeters(10);

        @Getter private double intakeDeployY = Units.inchesToMeters(3);
        @Getter private double length = Units.inchesToMeters(25);
        @Getter @Setter private double simRatio = 1;

        public IntakeDeployConfig() {
            super("IntakeDeploy", 35, Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(sensorToMechanismRatio);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-1 * torqueCurrentLimit);
            configMinMaxRotations(-1, 0.5);
            configReverseSoftLimit(-1, true);
            configForwardSoftLimit(0.5, true);
            configNeutralBrakeMode(true);
            configClockwise_Positive();
            configGravityType(true);
            setSimRatio(sensorToMechanismRatio);
        }
    }

    // ------------------------------------------------------------------------
    // State machine
    // ------------------------------------------------------------------------
    public enum WantedState {
        HOME,
        STOPPED,
        GROUND_CORAL_INTAKE,
        HANDOFF,
        L1
    }

    public enum SystemState {
        HOME,
        STOPPED,
        GROUND_CORAL_INTAKE,
        HANDOFF,
        L1
    }

    private WantedState wantedState = WantedState.STOPPED;
    private SystemState systemState = SystemState.STOPPED;

    @Getter private IntakeDeployConfig config;
    @Getter private IntakeDeploySim sim;

    public IntakeDeploy(IntakeDeployConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) {
            setInitialPosition();
        }

        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    // ------------------------------------------------------------------------
    // Periodic
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
            case GROUND_CORAL_INTAKE:
                return SystemState.GROUND_CORAL_INTAKE;
            case HANDOFF:
                return SystemState.HANDOFF;
            case L1:
                return SystemState.L1;
            default:
                return systemState;
        }
    }

    private void applyStates() {
        double wantedDegrees = 0.0;
        switch (systemState) {
            case HOME:
                wantedDegrees = 0.0;
                break;
            case STOPPED:
                stop();
                return;
            case GROUND_CORAL_INTAKE:
                wantedDegrees = 110.0;
                break;
            case HANDOFF:
                wantedDegrees = -30.0;
                break;
            case L1:
                wantedDegrees = 8.0;
                break;
            default:
                return;
        }
        final double finalWantedDegrees = wantedDegrees;
        setMMPositionFoc(getOffsetRotations(() -> finalWantedDegrees));
    }

    // ------------------------------------------------------------------------
    // Telemetry
    // ------------------------------------------------------------------------
    private void logTelemetry() {
        Telemetry.log("IntakeDeploy/SystemState", systemState.toString());
        Telemetry.log("IntakeDeploy/WantedState", wantedState.toString());
        Telemetry.log(
                "IntakeDeploy/PositionDegrees", getPositionDegrees() - config.getOffset(), "deg");
        Telemetry.log("IntakeDeploy/Voltage", getVoltage(), "volts");
        Telemetry.log("IntakeDeploy/StatorCurrent", getStatorCurrent(), "amps");
        Telemetry.log("IntakeDeploy/SupplyCurrent", getSupplyCurrent(), "amps");
        Telemetry.log("IntakeDeploy/Temp", getTemp(), "deg_C");
    }

    // ------------------------------------------------------------------------
    // Public helpers
    // ------------------------------------------------------------------------
    public void setWantedState(WantedState state) {
        wantedState = state;
    }

    public WantedState getWantedState() {
        return wantedState;
    }

    public SystemState getSystemState() {
        return systemState;
    }

    public boolean isAtSetpoint() {
        return isAtTargetPosition(config::getToleranceRotations);
    }

    public boolean isAtHandoffSetpoint() {
        return isAtRotations(getOffsetRotations(() -> -30.0), config::getToleranceRotations);
    }

    public DoubleSupplier getOffsetRotations(DoubleSupplier degrees) {
        return () -> degreesToRotations(offsetPosition(degrees));
    }

    public DoubleSupplier offsetPosition(DoubleSupplier position) {
        return () -> (position.getAsDouble() + config.getOffset());
    }

    // ----------------------------------------------------------------------------
    // Private helpers
    // ---------------------------------------------------------------------------
    void setInitialPosition() {
        motor.setPosition(degreesToRotations(offsetPosition(() -> config.getInitPosition())));
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    void simulationInit() {
        if (isAttached()) {
            sim = new IntakeDeploySim(motor.getSimState(), RobotSim.frontView);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
        }
    }

    class IntakeDeploySim extends ArmSim {
        public IntakeDeploySim(TalonFXSimState pivotMotorSim, Mechanism2d mech) {
            super(
                    new ArmConfig(
                            config.intakeDeployX,
                            config.intakeDeployY,
                            config.simRatio,
                            config.length,
                            -360,
                            360.0,
                            90),
                    mech,
                    pivotMotorSim,
                    "3" + config.getName());
        }
    }
}
