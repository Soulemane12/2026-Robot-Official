package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class ShooterAngleSubsystem extends SubsystemBase {
    private final TalonFX m_motor =
            new TalonFX(Constants.CANIds.SHOOTER_ANGLE_MOTOR, TunerConstants.kCANBus);

    private final MotionMagicVoltage m_mm = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut m_volts = new VoltageOut(0);

    public ShooterAngleSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = Constants.ShooterAngleConstants.STATOR_LIMIT_A;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = Constants.ShooterAngleConstants.SUPPLY_LIMIT_A;
        cfg.CurrentLimits.SupplyCurrentLowerLimit  = Constants.ShooterAngleConstants.SUPPLY_THRESHOLD_A;
        cfg.CurrentLimits.SupplyCurrentLowerTime   = Constants.ShooterAngleConstants.SUPPLY_TIME_S;

        cfg.Slot0.kP = Constants.ShooterAngleConstants.kP;
        cfg.Slot0.kD = Constants.ShooterAngleConstants.kD;
        cfg.Slot0.kS = Constants.ShooterAngleConstants.kS;
        cfg.Slot0.kV = Constants.ShooterAngleConstants.kV;

        cfg.MotionMagic.MotionMagicCruiseVelocity = Constants.ShooterAngleConstants.CRUISE_RPS;
        cfg.MotionMagic.MotionMagicAcceleration   = Constants.ShooterAngleConstants.ACCEL_RPS2;
        cfg.MotionMagic.MotionMagicJerk           = Constants.ShooterAngleConstants.JERK_RPS3;

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degToMotorRot(Constants.ShooterAngleConstants.MAX_DEG);
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degToMotorRot(Constants.ShooterAngleConstants.MIN_DEG);

        m_motor.getConfigurator().apply(cfg);

        // Boot assumption: hood is physically at MIN_DEG / Point A before power-on
        m_motor.setPosition(Constants.ShooterAngleConstants.ROT_A);
    }

    public void zeroHere() {
        m_motor.setPosition(Constants.ShooterAngleConstants.ROT_A);
    }

    public double getMotorRot() {
        return m_motor.getPosition().getValueAsDouble();
    }

    public double getAngleDeg() {
        return motorRotToDeg(getMotorRot());
    }

    public boolean atAngle(double targetDeg) {
        return Math.abs(getAngleDeg() - targetDeg) <= Constants.ShooterAngleConstants.ANGLE_TOLERANCE_DEG;
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public void setAngleDeg(double targetDeg) {
        double clamped = clamp(targetDeg, Constants.ShooterAngleConstants.MIN_DEG, Constants.ShooterAngleConstants.MAX_DEG);
        m_motor.setControl(m_mm.withPosition(degToMotorRot(clamped)));
    }

    public void jogVolts(double volts) {
        double angleDeg = getAngleDeg();
        boolean atMax = volts > 0.0 && angleDeg >= Constants.ShooterAngleConstants.MAX_DEG;
        boolean atMin = volts < 0.0 && angleDeg <= Constants.ShooterAngleConstants.MIN_DEG;
        m_motor.setControl(m_volts.withOutput((atMax || atMin) ? 0.0 : volts));
    }

    private static double degToMotorRot(double deg) {
        return Constants.ShooterAngleConstants.M_ROT_PER_DEG * deg
                + Constants.ShooterAngleConstants.B_ROT;
    }

    private static double motorRotToDeg(double rot) {
        return (rot - Constants.ShooterAngleConstants.B_ROT)
                / Constants.ShooterAngleConstants.M_ROT_PER_DEG;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ShooterAngle/Deg", getAngleDeg());
        SmartDashboard.putNumber("ShooterAngle/MotorRot", getMotorRot());
        SmartDashboard.putNumber("ShooterAngle/VelocityRPS", m_motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("ShooterAngle/SupplyCurrentA", m_motor.getSupplyCurrent().getValueAsDouble());
    }
}
