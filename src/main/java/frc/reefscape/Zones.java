package frc.reefscape;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;

public class Zones {
    @Getter private static final double atReefXYTolerance = Units.inchesToMeters(0.7); // 0.55

    @Getter
    private static final double atReefRotationTolerance = Units.degreesToRadians(0.35); // rads

    @Getter private static final double netAlgaeX = 9.618; // red coordinates
    @Getter private static final double netAlgaeZoneTolerance = 0.3;

    private static final Swerve swerve = Robot.getSwerveSubsystem();
    private static final double reefRangeRadius = Units.inchesToMeters(54.00);

    public static final Trigger blueFieldSide = swerve.inXzone(0, FieldConstants.fieldLength / 2);
    public static final Trigger opponentFieldSide =
            new Trigger(() -> blueFieldSide.getAsBoolean() != FieldConstants.isBlue());

    public static final Trigger topLeftZone =
            swerve.inXzoneAlliance(
                            FieldConstants.Reef.center.getX(), FieldConstants.fieldLength / 2)
                    .and(
                            swerve.inYzoneAlliance(
                                    FieldConstants.Reef.center.getY(), FieldConstants.fieldWidth));
    public static final Trigger topRightZone =
            swerve.inXzoneAlliance(
                            FieldConstants.Reef.center.getX(), FieldConstants.fieldLength / 2)
                    .and(swerve.inYzoneAlliance(0, FieldConstants.Reef.center.getY()));
    public static final Trigger bottomLeftZone =
            swerve.inXzoneAlliance(0, FieldConstants.Reef.center.getX())
                    .and(
                            swerve.inYzoneAlliance(
                                    FieldConstants.Reef.center.getY(), FieldConstants.fieldWidth));
    public static final Trigger bottomRightZone =
            swerve.inXzoneAlliance(0, FieldConstants.Reef.center.getX())
                    .and(swerve.inYzoneAlliance(0, FieldConstants.Reef.center.getY()));

    public static final Trigger netAlgaeZone =
            swerve.inXzone(netAlgaeX - netAlgaeZoneTolerance, netAlgaeX + netAlgaeZoneTolerance)
                    .or(
                            swerve.inXzone(
                                    (FieldConstants.fieldLength - netAlgaeX)
                                            - netAlgaeZoneTolerance,
                                    (FieldConstants.fieldLength - netAlgaeX)
                                            + netAlgaeZoneTolerance));

    public static final Trigger isCloseToReef =
            new Trigger(() -> Zones.withinReefRange(reefRangeRadius));

    public static boolean withinReefRange(double range) {
        Translation2d reefCenter = FieldHelpers.flipIfRedSide(FieldConstants.Reef.center);
        Pose2d robotPose = Robot.getSwerveSubsystem().getRobotPose();

        double distance = reefCenter.getDistance(robotPose.getTranslation());

        if (distance < range) {
            return true;
        }
        return false;
    }
}
