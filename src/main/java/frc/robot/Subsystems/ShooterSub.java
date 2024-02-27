// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSub extends SubsystemBase {

  private final CANSparkMax m_leftShooter = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax m_rightShooter = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax m_bottomShooter = new CANSparkMax(7, MotorType.kBrushless);

  // Creates UsbCamera and MjpegServer [1] and connects them
  UsbCamera usbCamera = new UsbCamera("USB Camera 0", 1);



  /** Creates a new ShooterSub. */
  public ShooterSub() {
    m_leftShooter.setInverted(true);
    m_rightShooter.setInverted(false);
    m_bottomShooter.setInverted(true);

    CameraServer.startAutomaticCapture();

    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



  public void shoot(double trigger) {
   m_leftShooter.set(-trigger);
   m_rightShooter.set(-trigger);

  }

  public void shootWithBool(double vel, boolean button) {
    if (button) {
      m_leftShooter.set(-vel);
      m_rightShooter.set(-vel);
    }
  }

  public void bottomShooterRunWBool(double vel, boolean button) {
    if (button) {
      m_bottomShooter.set(vel);
    }
  }




  public void setBrakeMode() {
    m_leftShooter.setIdleMode(IdleMode.kBrake);
    m_rightShooter.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode() {
    m_leftShooter.setIdleMode(IdleMode.kCoast);
    m_rightShooter.setIdleMode(IdleMode.kCoast);
  }

  public void setSmartCurrentLimit(int MaxOutput) {
    m_leftShooter.setSmartCurrentLimit(MaxOutput);
    m_rightShooter.setSmartCurrentLimit(MaxOutput);
  }

  public void stop() {
    m_leftShooter.stopMotor();
  }

  public void runBottomShooter(double trigger) {
    m_bottomShooter.set(trigger);
  }
}
