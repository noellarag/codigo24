// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSub extends SubsystemBase {

  private final CANSparkMax m_leftMaster = new CANSparkMax(3, MotorType.kBrushed);
  private final CANSparkMax m_leftFollower = new CANSparkMax(4, MotorType.kBrushed);
  private final CANSparkMax m_rightMaster = new CANSparkMax(1, MotorType.kBrushed);
  private final CANSparkMax m_rightFollower = new CANSparkMax(2, MotorType.kBrushed);

  private final DifferentialDrive m_drivetrain = new DifferentialDrive(m_leftMaster, m_rightMaster);

  private final static AHRS navx = new AHRS(SPI.Port.kMXP);

  private final Encoder m_leftEncoder = new Encoder(5, 6, false, EncodingType.k2X);
  private final Encoder m_rightEncoder = new Encoder(3, 4,true, EncodingType.k2X);
  boolean buttonPressedOnce;
  boolean buttonPressedTwice;


  DifferentialDriveOdometry m_odometry;
  /** Creates a new DrivetrainSub. */
  public DrivetrainSub() {

    m_rightMaster.setInverted(true);
    m_rightFollower.setInverted(true);

    m_odometry = new DifferentialDriveOdometry(navx.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    buttonPressedOnce = false;
    buttonPressedTwice = false;


    m_leftEncoder.setDistancePerPulse(Constants.kDriveTickToMeters);
    m_rightEncoder.setDistancePerPulse(Constants.kDriveTickToMeters);
    
    resetEncoders();
  }

  @Override
  public void periodic() {
    m_odometry.update(navx.getRotation2d(), m_leftEncoder.getDistance(),
    m_rightEncoder.getDistance());

    SmartDashboard.putNumber("Left encoder value meters", getLeftEncoderPosition());
    SmartDashboard.putNumber("RIGHT encoder value meters", getRightEncoderPosition());
    SmartDashboard.putNumber("Gyro heading", getHeading());
    SmartDashboard.putNumber("rate left", getLeftEncoderVelocity());
    SmartDashboard.putNumber("rate right", getRightEncoderVelocity());
    SmartDashboard.putBoolean("LowSpeed", buttonPressedOnce);
    SmartDashboard.putBoolean("MidSpeed", buttonPressedTwice);
    SmartDashboard.putBoolean("MaxSpeed", buttonPressedOnce == false && buttonPressedTwice == false);



    // This method will be called once per scheduler run
  }

  public void drive(double d, double e) {
    m_drivetrain.arcadeDrive(d, e);
  }

  public void setCoastMode() {
    m_leftMaster.setIdleMode(IdleMode.kCoast);
    m_leftFollower.setIdleMode(IdleMode.kCoast);
    m_rightMaster.setIdleMode(IdleMode.kCoast);
    m_rightFollower.setIdleMode(IdleMode.kCoast);
  }

  public void setBrakeMode() {
    m_leftMaster.setIdleMode(IdleMode.kBrake);
    m_leftFollower.setIdleMode(IdleMode.kBrake);
    m_rightMaster.setIdleMode(IdleMode.kBrake);
    m_rightFollower.setIdleMode(IdleMode.kBrake);  
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public void limitSpeed(double MaxSpeed, double MidSpeed, double LowSpeed, boolean button) {
    if (button) {
      if (buttonPressedTwice == true) {
        m_drivetrain.setMaxOutput(MaxSpeed);
        buttonPressedTwice = false;
      }
      else {
        if (buttonPressedOnce == true) {
          m_drivetrain.setMaxOutput(MidSpeed);
          buttonPressedOnce = false;
          buttonPressedTwice = true;
        }
        else {
          m_drivetrain.setMaxOutput(LowSpeed);
          buttonPressedOnce = true;
        }
      }
    }
  }

  public double get() {
    return m_leftMaster.get();
  }

  public double getRightEncoderPosition() {
    return m_rightEncoder.getDistance();
    //CHECK IF NEEDS TO BE INVERTED
  }

  public double getLeftEncoderPosition() {
    return m_leftEncoder.getDistance();
    //CHECK IF NEEDS TO BE INVERTED
  }

  public double getRightEncoderVelocity() {
    return m_rightEncoder.getRate();
  }

    public double getLeftEncoderVelocity() {
    return m_rightEncoder.getRate();
  }

  public double getTurnRate() {
    return navx.getRate();
  }
  public static double getHeading() {
    return navx.getRotation2d().getDegrees();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(navx.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), getPose());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }
  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMaster.setVoltage(leftVolts);
    m_rightMaster.setVoltage(rightVolts);
    m_leftFollower.setVoltage(leftVolts);
    m_rightFollower.setVoltage(rightVolts);    
    m_drivetrain.feed();
  }

  public double getAverageEncoderDistance() {
    return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0);
  }

  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
    m_drivetrain.setMaxOutput(maxOutput);
  }

  public static void zeroHeading() {
    navx.reset();
  }

  public AHRS getGyro() {
    return navx;
  }

  public DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  public void resetEverything(boolean button) {
    zeroHeading();
    resetEncoders();
  }

  public void setSmartLimit(int MaxOutput) {
    m_leftMaster.setSmartCurrentLimit(MaxOutput);
    m_rightMaster.setSmartCurrentLimit(MaxOutput);
    m_leftFollower.setSmartCurrentLimit(MaxOutput);
    m_rightFollower.setSmartCurrentLimit(MaxOutput);
  }






}
