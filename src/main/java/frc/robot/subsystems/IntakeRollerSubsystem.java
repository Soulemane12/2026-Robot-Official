package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRollerSubsystem extends SubsystemBase {
    private final TalonFX m_motor;
    private final VoltageOut m_voltageOut = new VoltageOut(0.0);
    private boolean m_running = false;

    public IntakeRollerSubsystem() {
        m_motor = new TalonFX(Constants.CANIds.INTAKE_ROLLER, frc.robot.generated.TunerConstants.kCANBus);
    }

    public void toggle() {
        m_running = !m_running;
        if (m_running) {
            m_motor.setControl(m_voltageOut.withOutput(Constants.IntakeConstants.ROLLER_VOLTAGE));
        } else {
            m_motor.setControl(m_voltageOut.withOutput(0.0));
        }
    }

    public void stop() {
        m_running = false;
        m_motor.setControl(m_voltageOut.withOutput(0.0));
    }

    public boolean isRunning() {
        return m_running;
    }
}
