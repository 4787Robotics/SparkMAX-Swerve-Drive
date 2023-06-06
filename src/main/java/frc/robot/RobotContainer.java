// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.SwerveCommand;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.TestSwerve;
import frc.robot.subsystems.TestSwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final TestSwerve testSwerve = new TestSwerve();

  //private final SwerveCommand m_autoCommand = new SwerveCommand(testSwerve);
  //test swerve turn
  
  //private final TestSwerveModule testSwerveModule = new TestSwerveModule(1, 2, 0, 0);
  //private final TurnToAngle turnToAngle = new TurnToAngle(30, testSwerveModule);
  private final TestSwerve testSwerve = new TestSwerve();
  XboxController m_driverController = new XboxController(0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    testSwerve.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
          () -> testSwerve.drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.05),
            MathUtil.applyDeadband(m_driverController.getLeftX(), 0.05),
            MathUtil.applyDeadband(m_driverController.getRightX(), 0.05)
            ),
          testSwerve));
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return turnToAngle;
  }*/
}
