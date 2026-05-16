package frc.reefscape;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class FieldHelpers {

    // -----------------------------------------------------------------------
    // Field Helper Methods
    // -----------------------------------------------------------------------

    /* Methods to flip robot pose */

    public static double flipAngle(double angle) {
        return (angle + 180) % 360;
    }

    public static Rotation2d flipAngle(Rotation2d angle) {
        return angle.rotateBy(Rotation2d.fromDegrees(180));
    }

    public static double flipAngleIfRed(double blue) {
        if (FieldConstants.isRed()) {
            return (blue + 180) % 360;
        }
        return blue;
    }

    public static Rotation2d flipAngleIfRed(Rotation2d blue) {
        if (FieldConstants.isRed()) {
            return blue.rotateBy(Rotation2d.fromDegrees(180));
        }
        return blue;
    }

    public static Translation2d flipIfRed(Translation2d blue) {
        return new Translation2d(flipXifRed(blue.getX()), flipYifRed(blue.getY()));
    }

    public static Translation3d flipIfRed(Translation3d blue) {
        return new Translation3d(flipXifRed(blue.getX()), flipYifRed(blue.getY()), blue.getZ());
    }

    public static Pose2d flipIfRed(Pose2d red) {
        return new Pose2d(flipIfRed(red.getTranslation()), flipAngleIfRed(red.getRotation()));
    }

    public static Translation2d flipIfRedSide(Translation2d red) {
        if (Zones.blueFieldSide.getAsBoolean()) {
            return red;
        }
        return new Translation2d(flipX(red.getX()), flipY(red.getY()));
    }

    public static Pose2d flipIfRedSide(Pose2d red) {
        if (Zones.blueFieldSide.getAsBoolean()) {
            return red;
        }
        return new Pose2d(
                flipIfRedSide(new Translation2d(red.getX(), red.getY())),
                flipAngle(red.getRotation()));
    }

    public static double flipX(double xCoordinate) {
        return FieldConstants.fieldLength - xCoordinate;
    }

    public static double flipY(double yCoordinate) {
        return FieldConstants.fieldWidth - yCoordinate;
    }

    // If we are red flip the x pose to the other side of the field
    public static double flipXifRed(double xCoordinate) {
        if (FieldConstants.isRed()) {
            return FieldConstants.fieldLength - xCoordinate;
        }
        return xCoordinate;
    }

    // If we are red flip the y pose to the other side of the field
    public static double flipYifRed(double yCoordinate) {
        if (FieldConstants.isRed()) {
            return FieldConstants.fieldWidth - yCoordinate;
        }
        return yCoordinate;
    }

    public static boolean poseOutOfField(Pose2d pose2D) {
        double x = pose2D.getX();
        double y = pose2D.getY();
        return (x <= 0 || x >= FieldConstants.fieldLength)
                || (y <= 0 || y >= FieldConstants.fieldWidth);
    }

    public static boolean poseOutOfField(Pose3d pose3D) {
        return poseOutOfField(pose3D.toPose2d());
    }

    // -----------------------------------------------------------------------
    // Cage Helper Methods
    // -----------------------------------------------------------------------

    public static int indexOfSmallest(double[] array) {
        int indexOfSmallest = 0;
        double smallestIndex = array[indexOfSmallest];
        for (int i = 0; i < array.length; i++) {
            if (array[i] <= smallestIndex) {
                smallestIndex = array[i];
                indexOfSmallest = i;
            }
        }
        return indexOfSmallest;
    }

    // -----------------------------------------------------------------------
    // Reef Helper Methods
    // -----------------------------------------------------------------------

    /* Tag ID methods */

    /**
     * Converts an index to a reef tag ID
     *
     * @param index
     * @return
     */
    public static int indexToReefTagID(int index) {
        return index + 17;
    }

    /**
     * Converts a given Reef Tag Id into index form for center faces to pull from
     *
     * @param tagID
     * @return
     */
    public static int blueReefTagIDToIndex(int tagID) {

        // blue reef indexer
        if (tagID < 17 || tagID > 22 || tagID < 0) {
            return -1;
        }

        return tagID - 17;
    }

    /**
     * Converts a blue reef tag ID to a red reef tag ID
     *
     * @param blueTagID
     * @return
     */
    public static int blueToRedTagID(int blueTagID) {
        switch (blueTagID) {
            case 17:
                return 8;
            case 18:
                return 7;
            case 19:
                return 6;
            case 20:
                return 11;
            case 21:
                return 10;
            case 22:
                return 9;
            default:
                return blueTagID;
        }
    }

    public static int redToBlueTagID(int redTagID) {
        switch (redTagID) {
            case 8:
                return 17;
            case 7:
                return 18;
            case 6:
                return 19;
            case 11:
                return 20;
            case 10:
                return 21;
            case 9:
                return 22;
            default:
                return redTagID;
        }
    }

    /**
     * Returns the reef tag ID based on the robot's pose
     *
     * @param pose
     * @return
     */
    public static int getReefZoneTagID(Pose2d pose) {
        pose = flipIfRedSide(pose);
        int tag = indexToReefTagID(getReefZone(pose));

        if (!Zones.blueFieldSide.getAsBoolean()) {
            tag = blueToRedTagID(tag);
        }

        SmartDashboard.putNumber("Target ID getReefZone: ", tag);
        return tag;
    }

    /* Reef pose methods */

    /**
     * Returns the reef index zone based on the robot's pose changed to blue pose including the
     * center is consistently blue center
     *
     * @param pose
     * @return
     */
    public static int getReefZone(Pose2d pose) {
        Translation2d point = pose.getTranslation();
        Translation2d relativePoint = point.minus(FieldConstants.Reef.center);
        double angle = Math.atan2(relativePoint.getX(), relativePoint.getY()); // Standard atan2
        double distance = relativePoint.getNorm();

        // Normalize angle to be between 0 and 2*PI
        if (angle < 0) {
            angle += 2 * Math.PI;
        }

        // Check if the point is within the 4.5 meters radius
        if (distance > 4.5) {
            // System.out.println("Distance Error");
            return -1; // Outside the zones
        }

        // Determine the zone based on the angle
        double zoneAngle = Math.PI / 3; // 60 degrees per zone
        int index = (int) ((angle + Math.PI) / zoneAngle); // Convert angle to zone index

        return index % 6; // Modular for safety, definitely works without the modular just don't
        // remove it
    }

    /**
     * Returns the reef face pose based on the tag ID sent from either red or blue
     *
     * @param tagID
     * @return Pose2d of reef side
     */
    public static Pose2d getReefSideFromTagID(int faceIndex) {
        if (faceIndex < 0) {
            return Robot.getSwerveSubsystem().getRobotPose();
        }
        Pose2d reefFacePose = FieldConstants.Reef.centerFaces[blueReefTagIDToIndex(faceIndex)];

        if (FieldConstants.isRed()) {
            reefFacePose = flipIfRed(reefFacePose);
            return flipIfRed(reefFacePose);
        }

        return reefFacePose;
    }

    // ------------------------------------------------------------------------------
    // Calculation Functions
    // ------------------------------------------------------------------------------

    /**
     * Get the angle the robot should turn to based on the id the limelight is seeing.
     *
     * @return
     */
    public static double getReefTagAngle() {
        double[][] reefFrontAngles = {
            {17, 60}, {18, 0}, {19, -60}, {20, -120}, {21, 180}, {22, 120},
            {6, 120}, {7, 180}, {8, -120}, {9, -60}, {10, 0}, {11, 60}
        };

        int closestTag = Robot.getVisionSubsystem().getClosestTagID();
        boolean rearTag = Robot.getVisionSubsystem().isRearTagClosest();

        if (closestTag <= 0) {
            Pose2d currentPose = Robot.getSwerveSubsystem().getRobotPose();
            int tagID = FieldHelpers.getReefZoneTagID(currentPose);
            closestTag = tagID;
            rearTag = false;
        }

        for (int i = 0; i < reefFrontAngles.length; i++) {
            if (closestTag == reefFrontAngles[i][0]) {
                if (rearTag
                        || !Robot.getSwerveSubsystem().frontClosestToAngle(reefFrontAngles[i][1])) {
                    return Math.toRadians(reefFrontAngles[i][1] + 180);
                }
                return Math.toRadians(reefFrontAngles[i][1]);
            }
        }

        // Return current angle if no tag is found
        return Robot.getSwerveSubsystem().getRobotPose().getRotation().getRadians();
    }
}
