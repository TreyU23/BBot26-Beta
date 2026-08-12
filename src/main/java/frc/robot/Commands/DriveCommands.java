package frc.robot.Commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriverConstants;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class DriveCommands {
    public static final SwerveRequest.FieldCentric FIELD_CENTRIC = new SwerveRequest.FieldCentric()
        .withDeadband(DriverConstants.MAX_SPEED * DriverConstants.DRIVE_DEADBAND)
        .withRotationalDeadband(DriverConstants.MAX_ANGULAR_RATE * DriverConstants.ROTATION_DEADBAND)
        .withDriveRequestType(DriveRequestType.Velocity);
    
    public static final SwerveRequest.RobotCentric ROBOT_CENTRIC = new SwerveRequest.RobotCentric()
        .withDeadband(DriverConstants.MAX_SPEED * DriverConstants.DRIVE_DEADBAND)
        .withRotationalDeadband(DriverConstants.MAX_ANGULAR_RATE * DriverConstants.ROTATION_DEADBAND)
        .withDriveRequestType(DriveRequestType.Velocity);

    public static SlewRateLimiter m_slewX = new SlewRateLimiter(12.0);
    public static SlewRateLimiter m_slewY = new SlewRateLimiter(12.0);
    public static SlewRateLimiter m_slewRot = new SlewRateLimiter(36.0);

    private static double m_maximumSpeed  = DriverConstants.MAX_SPEED;
    private static double m_maximumAngularRate = DriverConstants.MAX_ANGULAR_RATE;
    
    public static void updateSlew(double x, double y, double r) {
        m_slewX = new SlewRateLimiter(x, -x, m_slewX.lastValue());
        m_slewY = new SlewRateLimiter(y, -y, m_slewY.lastValue());
        m_slewRot = new SlewRateLimiter(r, -r, m_slewRot.lastValue());
    }
        
    public static Command driveRequest(
                    CommandSwerveDrivetrain drivetrain,
                    DoubleSupplier forwardBack,
                    DoubleSupplier leftRight,
                    DoubleSupplier rotation, BooleanSupplier isFieldCentric) {

                return drivetrain.applyRequest(() ->{
                  
                    double transCurveAdjustment = adjustInputCurve(forwardBack.getAsDouble(), leftRight.getAsDouble(),0.9,0.1);
                    SmartDashboard.putNumber("TransCurveAdj", transCurveAdjustment);
                    double rotCurveAdjustment = adjustRotCurve(rotation.getAsDouble(), 0.9, 0.1);
                    SmartDashboard.putNumber("RotCurveAdj", rotCurveAdjustment);
                    
                    double xOutput = m_slewX.calculate(-forwardBack.getAsDouble() * m_maximumSpeed*transCurveAdjustment);
                    double yOutput = m_slewY.calculate(-leftRight.getAsDouble() * m_maximumSpeed*transCurveAdjustment);
                    double rotOutput = m_slewRot.calculate(-rotation.getAsDouble() * m_maximumAngularRate*rotCurveAdjustment);
                    
                    SmartDashboard.putBoolean("FieldCentric", isFieldCentric.getAsBoolean());

                    if (isFieldCentric.getAsBoolean()) {
                        return FIELD_CENTRIC.withVelocityX(xOutput).withVelocityY(yOutput).withRotationalRate(rotOutput);
                    } else {
                        return ROBOT_CENTRIC.withVelocityX(xOutput).withVelocityY(yOutput).withRotationalRate(rotOutput);
                    }
                });
    }

    public static Command resetFieldOrientation(CommandSwerveDrivetrain drivetrain) {
        return drivetrain.runOnce(drivetrain::seedFieldCentric);
    }

    private static double adjustInputCurve(double x, double y, double a, double b){
        double mag = Math.sqrt(Math.pow(x,2)+Math.pow(y,2));
        if(mag > 1.0) mag = 1.0;
        if(mag < 0.0) mag = 0.0;
        mag = a*Math.pow(mag, 3)+b*mag;
        return mag;
    }

    public static double adjustRotCurve(double r, double a, double b){
        return Math.abs(a*Math.pow(r, 3)+b*r);
    }

    public static void updateMAX(double Speed, double Rotation) {
        m_maximumSpeed = Speed;
        m_maximumAngularRate = Rotation;
    }
}