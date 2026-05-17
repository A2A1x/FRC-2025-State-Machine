package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

// General Sim principles
// Always move the root/origin to change it's display position
// Looking at the robot from the left view (right side of the robot)
public class RobotSim {
    public static final double heightMeters = Units.inchesToMeters(100.0);
    public static final double widthMeters = Units.inchesToMeters(75.0);

    public static final Translation2d origin = new Translation2d(0.0, 0.0);

    public static final Mechanism2d frontView = new Mechanism2d(widthMeters, heightMeters);
    public static final Mechanism2d leftView = new Mechanism2d(widthMeters, heightMeters);

    public RobotSim() {
        SmartDashboard.putData("LeftView", RobotSim.leftView);
        SmartDashboard.putData("FrontView", RobotSim.frontView);
        leftView.setBackgroundColor(new Color8Bit(Color.kLightGray));
        frontView.setBackgroundColor(new Color8Bit(Color.kLightGray));
    }
}
