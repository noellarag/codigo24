// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSub;
import frc.robot.Subsystems.HookSub;

public class DrivetrainNHookingCmd extends Command {
  private final DrivetrainSub m_drivetrain;
  private final HookSub m_hookMotors;
  private final XboxController m_controller = new XboxController(0);
  /** Creates a new DrivetrainNHookingCmd. */
  public DrivetrainNHookingCmd(DrivetrainSub m_drivetrain, HookSub m_hookMotors) 
  {
    this.m_drivetrain = m_drivetrain;
    this.m_hookMotors = m_hookMotors;
    addRequirements(m_drivetrain);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.drive(0, 0);
    m_drivetrain.resetEncoders();
    m_drivetrain.resetEverything(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.getRightX() < 0.01 && m_drivetrain.get() < 0.02) {
      m_hookMotors.hookWDouble(m_controller.getRightY());
    }
  m_drivetrain.drive(m_controller.getLeftY(), m_controller.getRightX());
  m_drivetrain.limitSpeed(1.0, 0.5, 0.3, m_controller.getLeftStickButtonReleased());
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
