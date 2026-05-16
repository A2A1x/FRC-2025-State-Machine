package frc.robot.subsystems.shoulder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumCANcoder;
import frc.spectrumLib.SpectrumCANcoderConfig;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import java.util.function.DoubleSupplier;
import lombok.*;

public class Shoulder extends Mechanism {

    public static class ShoulderConfig extends Config {
        @Getter @Setter private boolean isPhoton = false;

        // Positions set as degrees of rotation || 0 is vertical up
        @Getter private final int initializedPosition = 0;

        @Getter private final double scoreDelay = 0.3;
        @Getter private final double prescoreDelay = 0.3;

        @Getter @Setter private double triggerTolerance = 3;

        @Getter @Setter private double offset = 90;
        @Getter @Setter private double initPosition = 0;

        /* Shoulder config settings */
        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18.0;

        @Getter private final double currentLimit = 60; // 60;
        @Getter private final double torqueCurrentLimit = 120; // 80;
        @Getter private final double positionKp = 250;
        @Getter private final double positionKd = 120;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 0;
        @Getter private final double positionKa = 0;
        @Getter private final double positionKg = 11;
        @Getter private final double mmCruiseVelocity = 10;
        @Getter private final double mmAcceleration = 7;
        @Getter private final double mmJerk = 0;

        @Getter @Setter private double sensorToMechanismRatio = 101.25;
        @Getter @Setter private double rotorToSensorRatio = 1;

        /* Cancoder config settings */
        @Getter @Setter private double CANcoderRotorToSensorRatio = 101.25 / 5;

        @Getter @Setter private double CANcoderSensorToMechanismRatio = 5;

        @Getter @Setter private double CANcoderOffset = 0;
        @Getter @Setter private boolean CANcoderAttached = false;

        /* Sim properties */
        @Getter private double shoulderX = RobotSim.widthMeters / 2;
        @Getter private double shoulderY = Units.inchesToMeters(7);
        @Getter private double length = Units.inchesToMeters(35);

        @Getter @Setter private double simRatio = 1;

        public ShoulderConfig() {
            super("Shoulder", 45, Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(sensorToMechanismRatio);
            configSupplyCurrentLimit(currentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-1 * torqueCurrentLimit);
            configMinMaxRotations(-1, 1);
            configReverseSoftLimit(-1, true);
            configForwardSoftLimit(1, true);
            configNeutralBrakeMode(true);
            if (Robot.isSimulation()) {
                configCounterClockwise_Positive();
            } else {
                configCounterClockwise_Positive();
            }
            configGravityType(true);
            setSimRatio(sensorToMechanismRatio);
        }

        public ShoulderConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    protected ShoulderConfig config;
    protected SpectrumCANcoder canCoder;
    protected SpectrumCANcoderConfig canCoderConfig;
    @Getter private ShoulderSim sim;
    CANcoderSimState canCoderSim;

    public Shoulder(ShoulderConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) { // && RobotStates.pm.and(RobotStates.photon,
            // RobotStates.sim).not().getAsBoolean()) {
            if (config.isCANcoderAttached() && !Robot.isSimulation()) {
                canCoderConfig =
                        new SpectrumCANcoderConfig(
                                config.getCANcoderRotorToSensorRatio(),
                                config.getCANcoderSensorToMechanismRatio(),
                                config.getCANcoderOffset(),
                                config.isCANcoderAttached());
                canCoder =
                        new SpectrumCANcoder(
                                46,
                                canCoderConfig,
                                motor,
                                config,
                                SpectrumCANcoder.CANCoderFeedbackType.FusedCANcoder);
            }

            setInitialPosition();
        }

        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    // ------------------------------------------------------------------------
    // State machine
    // ------------------------------------------------------------------------
    public enum WantedState {
        HOME,
        STOPPED,

        ALGAE_GROUND_INTAKE,
        CORAL_GROUND_INTAKE,
        CORAL_INTAKE_LOLIPOP,

        CLIMB_PREP,

        ALGAE_PROCESSOR,
        ALGAE_INTAKE_L2,
        ALGAE_INTAKE_L3,
        ALGAE_NET,

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
        CORAL_L4_RELEASE
    }

    public enum SystemState {
        HOME,
        STOPPED,

        ALGAE_GROUND_INTAKE,
        CORAL_GROUND_INTAKE,
        CORAL_INTAKE_LOLIPOP,

        CLIMB_PREP,

        ALGAE_PROCESSOR,
        ALGAE_INTAKE_L2,
        ALGAE_INTAKE_L3,
        ALGAE_NET,

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
        CORAL_L4_RELEASE
    }

    private WantedState wantedState = WantedState.STOPPED;
    private SystemState systemState = SystemState.STOPPED;

    // ------------------------------------------------------------------------
    // Periodic
    // ------------------------------------------------------------------------

    @Override
    public void periodic() {
        logTelemetry();

        systemState = handleStateTransition();
        applyStates();
    }

    // ------------------------------------------------------------------------
    // State transition logic
    // ------------------------------------------------------------------------

    private SystemState handleStateTransition() {
        switch (wantedState) {
            case HOME:
                return SystemState.HOME;
            case STOPPED:
                return SystemState.STOPPED;
            case STOWED_CORAL:
                return SystemState.STOWED_CORAL;
            case STOWED_ALGAE:
                return SystemState.STOWED_ALGAE;
            case ALGAE_GROUND_INTAKE:
                return SystemState.ALGAE_GROUND_INTAKE;
            case CORAL_GROUND_INTAKE:
                return SystemState.CORAL_GROUND_INTAKE;
            case CORAL_INTAKE_LOLIPOP:
                return SystemState.CORAL_INTAKE_LOLIPOP;
            case CLIMB_PREP:
                return SystemState.CLIMB_PREP;
            case ALGAE_PROCESSOR:
                return SystemState.ALGAE_PROCESSOR;
            case ALGAE_INTAKE_L2:
                return SystemState.ALGAE_INTAKE_L2;
            case ALGAE_INTAKE_L3:
                return SystemState.ALGAE_INTAKE_L3;
            case ALGAE_NET:
                return SystemState.ALGAE_NET;
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
            case PRE_CORAL_HANDOFF:
                return SystemState.PRE_CORAL_HANDOFF;
            case HANDOFF:
                return SystemState.HANDOFF;
            default:
                return systemState;
        }
    }

    // ------------------------------------------------------------------------
    // Apply states
    // ------------------------------------------------------------------------
    private void applyStates() {
        double wantedDegrees = 0.0;
        switch (systemState) {
            case HOME:
                wantedDegrees = 0;
                break;
            case STOPPED:
                stop();
                return;
            case STOWED_CORAL:
                wantedDegrees = 0;
                break;
            case STOWED_ALGAE:
                wantedDegrees = 0;
                break;
            case ALGAE_GROUND_INTAKE:
                wantedDegrees = -125;
                break;
            case CORAL_GROUND_INTAKE:
                wantedDegrees = 4;
                break;
            case CORAL_INTAKE_LOLIPOP:
                wantedDegrees = -110;
                break;
            case CLIMB_PREP:
                wantedDegrees = -100;
                break;
            case ALGAE_PROCESSOR:
                wantedDegrees = -143.877;
                break;
            case ALGAE_INTAKE_L2:
                wantedDegrees = -88;
                break;
            case ALGAE_INTAKE_L3:
                wantedDegrees = -88;
                break;
            case ALGAE_NET:
                wantedDegrees = -19;
                break;
            case CORAL_L2_LINEUP:
                wantedDegrees = -58;
                break;
            case CORAL_L2_RELEASE:
                wantedDegrees = -77.8;
                break;
            case CORAL_L3_LINEUP:
                wantedDegrees = -56.2;
                break;
            case CORAL_L3_RELEASE:
                wantedDegrees = -77.8;
                break;
            case CORAL_L4_LINEUP:
                wantedDegrees = -55;
                break;
            case CORAL_L4_RELEASE:
                wantedDegrees = -82.4;
                break;
            case PRE_CORAL_HANDOFF:
                wantedDegrees = -180;
                break;
            case HANDOFF:
                wantedDegrees = -180;
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
        Telemetry.log("Shoulder/SystemState", systemState.toString());
        Telemetry.log("Shoulder/WantedState", wantedState.toString());
        Telemetry.log("Shoulder/PositionDegrees", getPositionDegrees() - config.getOffset(), "deg");
        Telemetry.log("Shoulder/Voltage", getVoltage(), "volts");
        Telemetry.log("Shoulder/StatorCurrent", getStatorCurrent(), "amps");
        Telemetry.log("Shoulder/SupplyCurrent", getSupplyCurrent(), "amps");
        Telemetry.log("Shoulder/Temp", getTemp(), "deg_C");
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
        return isAtTargetPosition(config::getTriggerTolerance);
    }

    public double checkMoveOverTop(DoubleSupplier degrees) {
        double newDeg = degrees.getAsDouble();
        if (newDeg < -90 && (getPositionDegrees() - config.offset) > 90) {
            newDeg += 360;
        } else if (newDeg > 90 && (getPositionDegrees() - config.offset) < -90) {
            newDeg -= 360;
        }
        return newDeg;
    }

    public DoubleSupplier getOffsetRotations(DoubleSupplier degrees) {
        return () -> degreesToRotations(offsetPosition(() -> checkMoveOverTop(degrees)));
    }

    public DoubleSupplier offsetPosition(DoubleSupplier position) {
        return () -> (position.getAsDouble() + config.getOffset());
    }

    // ----------------------------------------------------------------------------
    // Private helpers
    // ---------------------------------------------------------------------------

    void setInitialPosition() {
        if (canCoder != null) {
            if (canCoder.isAttached()
                    && canCoder.canCoderResponseOK(
                            canCoder.getCanCoder().getAbsolutePosition().getStatus())) {
                motor.setPosition(
                        canCoder.getCanCoder().getAbsolutePosition().getValueAsDouble()
                                / config.getCANcoderSensorToMechanismRatio());
            } else {
                motor.setPosition(
                        degreesToRotations(offsetPosition(() -> config.getInitPosition())));
            }
        } else {
            motor.setPosition(degreesToRotations(offsetPosition(() -> config.getInitPosition())));
        }
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    void simulationInit() {
        if (isAttached()) {
            sim = new ShoulderSim(motor.getSimState(), RobotSim.leftView);
            // m_CANcoder.setPosition(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
            // m_CANcoder.getSimState().setRawPosition(sim.getAngleRads() / 0.202);
        }
    }

    class ShoulderSim extends ArmSim {
        public ShoulderSim(TalonFXSimState shoulderMotorSim, Mechanism2d mech) {
            super(
                    new ArmConfig(
                                    config.shoulderX,
                                    config.shoulderY,
                                    config.simRatio,
                                    config.length,
                                    -360,
                                    360.0,
                                    90)
                            .setMount(Robot.getElevatorSubsystem().getSim(), true),
                    mech,
                    shoulderMotorSim,
                    "2" + config.getName()); // added 2 to the name to create it second
        }
    }
}
