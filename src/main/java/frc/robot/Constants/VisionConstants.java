package frc.robot.Constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class VisionConstants {

    public static final String front = "limelight-front";
    public static final String left = "limelight-left";
    public static final String right = "limelight-right";

    public static final InterpolatingDoubleTreeMap MultTag_N1_2 = new InterpolatingDoubleTreeMap(); //XY
    public static final InterpolatingDoubleTreeMap MultTag_N3 = new InterpolatingDoubleTreeMap(); //Rotation
    public static final InterpolatingDoubleTreeMap megaTag2TreeMap = new InterpolatingDoubleTreeMap(); //XY & Rotation

    public static void SetUpTrustConstants() {
        MultTag_N1_2.put(0.2, 100.0);
        MultTag_N1_2.put(0.25, 4.0);
        MultTag_N1_2.put(0.3, 2.0);
        MultTag_N1_2.put(0.6, 0.25);

        MultTag_N3.put(0.05, 100.0);
        MultTag_N3.put(0.25, 5.0);
        MultTag_N3.put(0.3, 2.0);
        MultTag_N3.put(0.6, 0.5);

        megaTag2TreeMap.put(0.05, 10.0);
        megaTag2TreeMap.put(0.1, 5.0);
        megaTag2TreeMap.put(0.2, 2.5);
        megaTag2TreeMap.put(0.25, 1.0);
        megaTag2TreeMap.put(0.3, 0.5);
        megaTag2TreeMap.put(0.6, 0.25);
    }
}