// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSub;

public class DrivetrainCmd extends Command {
  private final DrivetrainSub drivetrain;
  private final XboxController xboxController = new XboxController(0);

  /** Creates a new DrivetrainCmd. */
  public DrivetrainCmd(DrivetrainSub drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.drive(0, 0);
    drivetrain.resetEncoders();
    drivetrain.resetEverything(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(xboxController.getLeftY(), xboxController.getRightX());
    
    drivetrain.limitSpeed(1.0, 0.5, 0.3, xboxController.getLeftStickButtonReleased());
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
