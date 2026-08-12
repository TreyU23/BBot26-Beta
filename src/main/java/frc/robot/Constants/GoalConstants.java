package frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class GoalConstants {
    private static final double kOutpostFromSideWallInches = 72.0;
    private static final double kOutpostFromAllianceWallInches = 96.0;
    private static final double kRampFromSideWallInches = 60.0;
    private static final double kRampFromAllianceWallInches = 320.0;

    public static final class Blue {
        public static final Translation2d kHub = new Translation2d(182.11 / 39.37, 158.84 / 39.37);
        public static final Translation2d kFeedOutpost = new Translation2d(
                kOutpostFromAllianceWallInches / 39.37,
                kOutpostFromSideWallInches / 39.37);
        public static final Translation2d kFeedDepo = new Translation2d(
                kOutpostFromAllianceWallInches / 39.37,
                (317.69 - kOutpostFromSideWallInches) / 39.37);
        public static final Pose2d kTower = new Pose2d(new Translation2d(1.491 ,3.708), new Rotation2d(0.0));
        public static final Translation2d kFeedDepoRamp = new Translation2d(
                kRampFromAllianceWallInches / 39.37,
                (317.69 - kRampFromSideWallInches) / 39.37);;
                public static final Translation2d kFeedOutpostRamp = new Translation2d(
                kRampFromAllianceWallInches / 39.37,
                kRampFromSideWallInches / 39.37);;

    }

    public static final class Red {
        public static final Translation2d kHub = new Translation2d((651.22 - 182.11) / 39.37, 158.84 / 39.37);
        public static final Translation2d kFeedOutpost = new Translation2d(
                (651.22 - kOutpostFromAllianceWallInches) / 39.37,
                (317.69 - kOutpostFromSideWallInches) / 39.37);
        public static final Translation2d kFeedDepo = new Translation2d(
                (651.22 - kOutpostFromAllianceWallInches) / 39.37,
                kOutpostFromSideWallInches / 39.37);
        public static final Pose2d kTower = new Pose2d(new Translation2d(15.030 ,4.314), new Rotation2d(Math.PI));
                public static final Translation2d kFeedDepoRamp = new Translation2d(
                (651.22 - kRampFromAllianceWallInches) / 39.37,
                kRampFromSideWallInches / 39.37);;
                public static final Translation2d kFeedOutpostRamp = new Translation2d(
                (651.22 - kRampFromAllianceWallInches) / 39.37,
                (317.69 - kRampFromSideWallInches) / 39.37);;
    }
}
