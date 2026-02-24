package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_motor;

    private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0.0);
    private final VoltageOut m_voltageOut = new VoltageOut(0.0);

    private double m_targetRotations = 0.0;

    public IntakeSubsystem() {
        m_motor = new TalonFX(Constants.CANIds.INTAKE_PIVOT);

        TalonFXConfiguration config = new TalonFXConfiguration();

        MotionMagicConfigs mmConfig = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(20.0)
            .withMotionMagicAcceleration(60.0)
            .withMotionMagicJerk(0.0);
        config.MotionMagic = mmConfig;

        m_motor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/PosRot", getPositionRotations());
        SmartDashboard.putNumber("Intake/TargetRot", m_targetRotations);
        SmartDashboard.putNumber("Intake/VelocityRPS", getVelocityRPS());
    }

    public void zero() {
        m_motor.setPosition(0.0);
        m_targetRotations = 0.0;
    }

    public void goTo(double rotations) {
        m_targetRotations = rotations;
        m_motor.setControl(m_motionMagic.withPosition(rotations));
    }

    public void jogVolts(double volts) {
        m_motor.setControl(m_voltageOut.withOutput(volts));
    }

    public void stop() {
        m_motor.setControl(m_voltageOut.withOutput(0.0));
    }

    public double getPositionRotations() {
        return m_motor.getPosition().getValueAsDouble();
    }

    public double getVelocityRPS() {
        return m_motor.getVelocity().getValueAsDouble();
    }
}
