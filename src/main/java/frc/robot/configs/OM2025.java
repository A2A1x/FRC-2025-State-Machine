package frc.robot.configs;

import frc.robot.Robot.Config;

public class OM2025 extends Config {

    public OM2025() {
        super();
        swerve.configEncoderOffsets(-0.257324, 0.087891, 0.371582, 0.448730);
        // swerve.configEncoderOffsets(-0.395264, 0.11499, 0.061279, 0.385742);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        elevator.setAttached(true);
    }
}
