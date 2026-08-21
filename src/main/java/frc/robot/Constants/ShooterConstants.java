package frc.robot.Constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    public static final InterpolatingDoubleTreeMap kPositionTable = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kVelocityTable = new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap kPositionFeedTable = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kVelocityFeedTable = new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap kTimeTable = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kFeedTimeTable = new InterpolatingDoubleTreeMap();
    
    public class Hood {
        public static final int kMotorID = 12;

        public static final double kForwardLimit = 0.0;
        public static final double kReverseLimit = 0.0;

        public static final double kP = 0.0;
        public static final double kV = 0.0;
        public static final double kS = 0.0;
    }
    
    public class Roller {
        public static final int kMotorID = 13;

        public static final double kP = 0.0;
        public static final double kV = 0.0;
        public static final double kS = 0.0;
    }

    public static void setUpDistanceTables() {
        kPositionTable.put(0.0, 0.0);

        kVelocityTable.put(0.0, 0.0);

        kPositionFeedTable.put(0.0, 0.0);

        kVelocityFeedTable.put(0.0, 0.0);

        kTimeTable.put(0.0, 0.0);

        kFeedTimeTable.put(0.0, 0.0);
    }
}
