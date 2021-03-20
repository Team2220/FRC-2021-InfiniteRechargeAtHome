// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  public final DriveTrain driveTrain = new DriveTrain();

  public Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));
    config.setKinematics(driveTrain.getKinematics());
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory( 
      Arrays.asList(new Pose2d(), new Pose2d(1, 1, new Rotation2d(Math.PI))),
       config
    );
   
    FunctionalCommand runCommand = new FunctionalCommand(
      ()->{driveTrain.resetGyro();}, 
      ()->{},
      (interrupted)->{},
      ()->{return true;},
      driveTrain
    );

    WaitCommand waitCommand = new WaitCommand(2);


    RamseteCommand command = new DebugRamseteCommand(
      trajectory, 
      driveTrain::getPose, 
      new RamseteController(2.0, 0.7),
      driveTrain.getFeedforward(), 
      driveTrain.getKinematics(), 
      driveTrain::getSpeeds,
      driveTrain.getLeftPIDController(), 
      driveTrain.getRightPIDController(), 
      driveTrain::setOutput, 
      driveTrain
      );

    return (runCommand).andThen(waitCommand).andThen(command).andThen(()->{ 
      driveTrain.setOutput(0, 0);
    }, 
    driveTrain);
  }

  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
