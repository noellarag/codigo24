//Name by: Ale

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HookSub extends SubsystemBase {
  private final CANSparkMax m_hookMotor1 = new CANSparkMax(9, MotorType.kBrushed);
  private final CANSparkMax m_hookMotor2 = new CANSparkMax(8, MotorType.kBrushed);
  /** Creates a new HookingSub. */
  public HookSub() {
    m_hookMotor1.setInverted(false);
    m_hookMotor2.setInverted(false);
  }

  public void hook(double vel, boolean button) {
    if (button) {
      m_hookMotor1.set(vel);
      m_hookMotor2.set(vel);
    }
  }

  public void hookWDouble(double trigger) {
    m_hookMotor1.set(trigger);
    m_hookMotor2.set(trigger);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
