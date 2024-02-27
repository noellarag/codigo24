// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.DrivetrainCmd;
import frc.robot.Commands.DrivetrainNHookingCmd;
import frc.robot.Commands.HookingCmd;
import frc.robot.Commands.IntakeCmd;
import frc.robot.Commands.ShooterCmd;
import frc.robot.Subsystems.DrivetrainSub;
import frc.robot.Subsystems.HookSub;
import frc.robot.Subsystems.IntakeSub;
import frc.robot.Subsystems.ShooterSub;

public class RobotContainer {
  private final DrivetrainSub m_drivetrain = new DrivetrainSub();
  private final ShooterSub m_shooter = new ShooterSub();
  private final IntakeSub m_intake = new IntakeSub();
  private final HookSub m_hookMotors = new HookSub();

  Trajectory trajectory = new Trajectory();
  SendableChooser<Command> chooser = new SendableChooser<>();
  public RobotContainer() {
    configureBindings();
    chooser.addOption("3 meters", loadPathplannerTrajectoryToRamseteCommand(
        "pathplanner/paths/3m.path",
        true));
    chooser.addOption("Right to Middle", loadPathplannerTrajectoryToRamseteCommand(
        "pathplanner/RightToMiddle.path",
        true));

    Shuffleboard.getTab("Autonomous").add(chooser);

  }

  private void configureBindings() {}

  public Command loadPathplannerTrajectoryToRamseteCommand(String filename, boolean resetOdomtry) {


    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException exception) {
      DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
      System.out.println("Unable to read from file " + filename);
      return new InstantCommand();
    }

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_drivetrain::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics, m_drivetrain::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0), m_drivetrain::tankDriveVolts,
        m_drivetrain);

    if (resetOdomtry) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> m_drivetrain.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
    } else {
      return ramseteCommand;
    }

  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public Command getTeleopCommand() {
    return new ParallelCommandGroup(new DrivetrainNHookingCmd(m_drivetrain, m_hookMotors), new ShooterCmd(m_shooter), new IntakeCmd(m_intake));
  }
}
