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

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX m_motor =
            new TalonFX(Constants.CANIds.CLIMBER_MOTOR, TunerConstants.kCANBus);

    private final VoltageOut m_volts = new VoltageOut(0.0);
    private final MotionMagicVoltage m_mm = new MotionMagicVoltage(0.0).withSlot(0);

    public ClimberSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.Slot0.kP = Constants.ClimberConstants.kP;
        cfg.Slot0.kD = Constants.ClimberConstants.kD;
        cfg.Slot0.kS = Constants.ClimberConstants.kS;
        cfg.Slot0.kV = Constants.ClimberConstants.kV;

        cfg.MotionMagic.MotionMagicCruiseVelocity = Constants.ClimberConstants.CRUISE_RPS;
        cfg.MotionMagic.MotionMagicAcceleration   = Constants.ClimberConstants.ACCEL_RPS2;
        cfg.MotionMagic.MotionMagicJerk           = Constants.ClimberConstants.JERK_RPS3;

        m_motor.getConfigurator().apply(cfg);

        m_motor.setPosition(0.0);
    }

    /** Zero encoder — run with the climber physically at the stowed position. */
    public void zero() {
        m_motor.setPosition(0.0);
    }

    /** Go to a position in motor rotations using MotionMagic. */
    public void goTo(double rotations) {
        m_motor.setControl(m_mm.withPosition(rotations));
    }

    /** Jog at a fixed voltage — use to find positions manually. */
    public void extend() {
        m_motor.setControl(m_volts.withOutput(Constants.ClimberConstants.CLIMB_VOLTAGE));
    }

    public void retract() {
        m_motor.setControl(m_volts.withOutput(-Constants.ClimberConstants.CLIMB_VOLTAGE));
    }

    public void stop() {
        m_motor.setControl(m_volts.withOutput(0.0));
    }

    public double getPositionRotations() {
        return m_motor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber/PosRot", getPositionRotations());
        SmartDashboard.putNumber("Climber/SupplyCurrentA", m_motor.getSupplyCurrent().getValueAsDouble());
    }
}
