package frc.robot.pilot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.gamepads.Gamepad;
import lombok.Getter;
import lombok.Setter;

/* A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simulation */
public class Pilot extends Gamepad {
    public final Trigger LB = leftBumper;
    public final Trigger RB = rightBumper;
    public final Trigger LT = leftTrigger;
    public final Trigger RT = rightTrigger;

    public final Trigger AButton = A;
    public final Trigger BButton = B;
    public final Trigger XButton = X;
    public final Trigger YButton = Y;

    public final Trigger startButton = start;
    public final Trigger selectButton = select;

    public final Trigger leftStickPress = leftStickClick;
    public final Trigger rightStickPress = rightStickClick;

    public final Trigger dPadUp = upDpad;
    public final Trigger dPadDown = downDpad;
    public final Trigger dPadLeft = leftDpad;
    public final Trigger dPadRight = rightDpad;

    public static class PilotConfig extends Config {

        @Getter @Setter private double slowModeScalor = 0.45;
        @Getter @Setter private double defaultTurnScalor = 0.6;
        @Getter @Setter private double turboModeScalor = 1;
        private double deadzone = 0.05;

        public PilotConfig() {
            super("Pilot", 0);

            setLeftStickDeadzone(deadzone);
            setLeftStickExp(3);
            // Set Scalar in Constructor from Swerve Config

            setRightStickDeadzone(deadzone);
            setRightStickExp(3.0);
            setRightStickScalar(3 * Math.PI);

            setTriggersDeadzone(deadzone);
            setTriggersExp(1);
            setTriggersScalar(1);
        }
    }

    private PilotConfig config;

    /** Create a new Pilot with the default name and port. */
    public Pilot(PilotConfig config) {
        super(config);
        this.config = config;

        // Set Left stick Scalar from Swerve Config
        config.setLeftStickScalar(Robot.getConfig().swerve.getSpeedAt12Volts().magnitude());
        leftStickCurve.setScalar(config.getLeftStickScalar());

        register();
        Telemetry.print("Pilot Subsystem Initialized: ");
    }

    public void setMaxVelocity(double maxVelocity) {
        leftStickCurve.setScalar(maxVelocity);
    }

    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        rightStickCurve.setScalar(maxRotationalVelocity);
    }

    // Positive is forward, up on the left stick is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveFwdPositive() {
        double fwdPositive = leftStickCurve.calculate(-1 * getLeftY());
        return fwdPositive;
    }

    // Positive is left, left on the left stick is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveLeftPositive() {
        double leftPositive = -1 * leftStickCurve.calculate(getLeftX());
        return leftPositive;
    }

    // Positive is counter-clockwise, left Trigger is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveCCWPositive() {
        double ccwPositive = rightStickCurve.calculate(getRightX());
        ccwPositive *= Math.abs(config.getDefaultTurnScalor());
        return -1 * ccwPositive; // invert the value
    }

    public double getPilotStickAngle() {
        return getLeftStickDirection().getRadians();
    }
}
