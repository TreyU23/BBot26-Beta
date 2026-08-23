package frc.robot.Subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.TurretConstants;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class Turret extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(TurretConstants.kMotorID);

    private final CANcoder m_leftCANCoder = new CANcoder(TurretConstants.kLeftCANCoderID);
    private final CANcoder m_rightCANCoder = new CANcoder(TurretConstants.kRightCANCoderID);

    private final PositionVoltage m_pvRequest = new PositionVoltage(0.0);

    private double m_setpoint = 0.0;
    private double m_lastSetpoint = 0.0;
    private boolean m_targetReachable = true;

    private DoubleSupplier m_feedForward;

    private EasyCRT m_easyCRT;
    private Angle m_CRTOutput = Rotations.of(0.0);

    public Turret(DoubleSupplier robotRotation) {
        m_feedForward = () -> (-robotRotation.getAsDouble() * 1.1 / (2.0 * Math.PI) * TurretConstants.kV);

        notSoEasyCRT();
        CANCoderConfigs();
        motorConfigs();
        setStartingPose();
    }

    public double getStatorCurrent() {
        return m_motor.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return m_motor.getSupplyCurrent().getValueAsDouble();
    }

    public double getPosition() {
        return m_motor.getPosition().getValueAsDouble();
    }

    public Translation2d getTranslation() {
        return TurretConstants.kTranlation;
    }

    public boolean atSetpoint() {
        return m_targetReachable
                && MathUtil.isNear(m_setpoint, getPosition(), TurretConstants.kTolerance);
    }

    public void setPosition(double Angle) {
        double wrappedSetpoint = wrapAngle(Angle / (2.0 * Math.PI));
        SetpointSelection selection = selectSetpoint(
                wrappedSetpoint,
                m_lastSetpoint,
                TurretConstants.kReverseLimit,
                TurretConstants.kForwardLimit);

        m_setpoint = selection.setpoint();
        m_targetReachable = selection.reachable();
    }

    public void stop() {
        m_setpoint = getPosition();
        m_motor.stopMotor();
    }

    @Override
    public void periodic() {
        m_motor.setControl(m_pvRequest.withPosition(m_setpoint)
                .withFeedForward(Voltage.ofBaseUnits(m_feedForward.getAsDouble(), Volts)));

        SmartDashboard.putNumber("Turret Stator Current", getStatorCurrent());
        SmartDashboard.putNumber("Turret Supply Current", getSupplyCurrent());
        SmartDashboard.putNumber("Turret Position", getPosition());
        SmartDashboard.putNumber("Turret Setpoint", m_setpoint);
        SmartDashboard.putBoolean("Turret Target Reachable", m_targetReachable);

        SmartDashboard.putNumber("Turret Left CANCoder Position",
                m_leftCANCoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Turret Right CANCoder Position",
                m_rightCANCoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Turret Left CANCoder Rel Position", m_leftCANCoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Turret Right CANCoder Rel Position",
                m_rightCANCoder.getPosition().getValueAsDouble());

        SignalLogger.writeDouble("Turret Setpoint", m_setpoint);
        SignalLogger.writeBoolean("Turret Target Reachable", m_targetReachable);

        m_lastSetpoint = m_setpoint;
    }

    public void notSoEasyCRT() {
        Supplier<Angle> leftCANCoderPose = () -> m_leftCANCoder.getAbsolutePosition().getValue();
        Supplier<Angle> RightCANCoderPose = () -> m_rightCANCoder.getAbsolutePosition().getValue();

        EasyCRTConfig Config = new EasyCRTConfig(leftCANCoderPose, RightCANCoderPose)
                .withEncoderRatios(TurretConstants.kCRTGearRatio1, TurretConstants.kCRTGearRatio2)
                .withMechanismRange(Rotations.of(TurretConstants.kReverseLimit),
                        Rotations.of(TurretConstants.kForwardLimit))
                .withAbsoluteEncoderInversions(false, false)
                .withMatchTolerance(Rotations.of(TurretConstants.kCRTTolerance));

        m_easyCRT = new EasyCRT(Config);
    }

    public void setStartingPose() {
        SmartDashboard.putBoolean("CRT Set", false);

        m_easyCRT.getAngleOptional().ifPresent(mechAngle -> {
            SmartDashboard.putNumber("CRT Solve", mechAngle.magnitude());
            m_CRTOutput = Rotations.of(mechAngle.in(Rotations) + TurretConstants.kStartingPoseOffset);
            SmartDashboard.putBoolean("CRT Set", true);
        });

        m_motor.getConfigurator().setPosition(m_CRTOutput);
        m_setpoint = getPosition();
    }

    public double wrapAngle(double angleRotations) {
        double w = angleRotations - Math.floor(angleRotations);
        if (w >= 0.5)
            w -= 1.0;
        return w;
    }

    public double chooseSetpoint(double wrappedSetpoint, double lastSetpoint) {
        return selectSetpoint(
                wrappedSetpoint,
                lastSetpoint,
                TurretConstants.kReverseLimit,
                TurretConstants.kForwardLimit).setpoint();
    }

    static SetpointSelection selectSetpoint(
            double wrappedSetpoint,
            double lastSetpoint,
            double reverseLimit,
            double forwardLimit) {
        double safeLastSetpoint = Double.isFinite(lastSetpoint) ? lastSetpoint : 0.0;

        if (!Double.isFinite(reverseLimit)
                || !Double.isFinite(forwardLimit)
                || reverseLimit > forwardLimit) {
            return new SetpointSelection(safeLastSetpoint, false);
        }

        safeLastSetpoint = MathUtil.clamp(safeLastSetpoint, reverseLimit, forwardLimit);
        if (!Double.isFinite(wrappedSetpoint)) {
            return new SetpointSelection(safeLastSetpoint, false);
        }

        // Any equivalent turret angle is wrappedSetpoint plus an integer number of
        // rotations. Determine the full integer range allowed by the soft limits rather
        // than considering only -1, 0, and 1 rotations.
        double minimumRotationOffset = Math.ceil(reverseLimit - wrappedSetpoint);
        double maximumRotationOffset = Math.floor(forwardLimit - wrappedSetpoint);

        if (minimumRotationOffset > maximumRotationOffset) {
            // The requested field angle is outside the turret's reachable arc. Hold the
            // last safe position and keep atSetpoint() false instead of sending NaN to
            // the motor controller or treating the fallback as valid aim.
            return new SetpointSelection(safeLastSetpoint, false);
        }

        double nearestRotationOffset = Math.rint(safeLastSetpoint - wrappedSetpoint);
        double selectedRotationOffset = MathUtil.clamp(
                nearestRotationOffset,
                minimumRotationOffset,
                maximumRotationOffset);
        double selectedSetpoint = MathUtil.clamp(
                wrappedSetpoint + selectedRotationOffset,
                reverseLimit,
                forwardLimit);

        return new SetpointSelection(selectedSetpoint, true);
    }

    static record SetpointSelection(double setpoint, boolean reachable) {
    }

    public void CANCoderConfigs() {
        m_leftCANCoder.getConfigurator().apply(new MagnetSensorConfigs()
                .withMagnetOffset(TurretConstants.kLeftMagnetOffset)
                .withAbsoluteSensorDiscontinuityPoint(1)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

        m_rightCANCoder.getConfigurator().apply(new MagnetSensorConfigs()
                .withMagnetOffset(TurretConstants.kRightMagnetOffset)
                .withAbsoluteSensorDiscontinuityPoint(1)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive));
    }

    public void motorConfigs() {
        TalonFXConfigurator config = m_motor.getConfigurator();

        config.apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CurrentLimitConstants.kTurretStator)
                .withSupplyCurrentLimit(CurrentLimitConstants.kTurretSupply)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true));

        config.apply(new Slot0Configs()
                .withKP(TurretConstants.kP));

        config.apply(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(TurretConstants.kForwardLimit)
                .withReverseSoftLimitThreshold(TurretConstants.kReverseLimit)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true));

        config.apply(new FeedbackConfigs()
                .withSensorToMechanismRatio(TurretConstants.kGearRatio)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

        config.apply(new VoltageConfigs()
                .withPeakForwardVoltage(TurretConstants.kPeakForwardVolts)
                .withPeakReverseVoltage(TurretConstants.kPeakReverseVolts));

        m_motor.setNeutralMode(NeutralModeValue.Brake);
    }
}
