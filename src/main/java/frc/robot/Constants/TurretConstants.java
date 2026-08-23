package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;

public class TurretConstants {
    public static final int kMotorID = 16;

    public static final int kLeftCANCoderID = 25;
    public static final int kRightCANCoderID = 26;
    
    public static final double kP = 0.0;
    public static final double kV = 0.0;

    public static final double kForwardLimit = 0.0;
    public static final double kReverseLimit = 0.0;

    public static final double kPeakForwardVolts = 0.0;
    public static final double kPeakReverseVolts = 0.0;

    public static final double kGearRatio = 0.0;
    public static final double kCRTGearRatio1 = 0.0;
    public static final double kCRTGearRatio2 = 0.0;

    public static final double kLeftMagnetOffset = 0.0;
    public static final double kRightMagnetOffset = 0.0;

    public static final double kCRTTolerance = 0.0;
    public static final double kTolerance = 0.0;

    // Raw CRT solve reads ~0.25 rotations left of forward at power-on; this corrects it
    // (90 degrees right) so straight-forward reads as 0.0 rotations. Flip the sign if the
    // turret doesn't read 0.0 with the barrel pointed forward after zeroing.
    public static final double kStartingPoseOffset = -0.25;

    public static final Translation2d kTranlation = new Translation2d(0,0);
}
