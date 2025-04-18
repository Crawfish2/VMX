/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.SimpleCamera;
// import frc.robot.subsystems.DepthCamera;
import frc.robot.subsystems.TitanKilloughDrive;
import frc.robot.subsystems.UltraSonicSensor;
import frc.robot.util.CommandTester;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final TitanKilloughDrive drive = new TitanKilloughDrive();
  private final UltraSonicSensor sonar = new UltraSonicSensor();
  private final SimpleCamera camera = new SimpleCamera();
  // public static final DepthCamera camera = new DepthCamera();
  private final CommandTester tester = new CommandTester(drive, sonar);

  private final PS4Controller controller = new PS4Controller(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // Set default commands
    final ShuffleboardTab tab = Shuffleboard.getTab("test");
    final var X = tab.add("X", 0).getEntry();
    final var Y = tab.add("Y", 0).getEntry();

    drive.setDefaultCommand(
        new RunCommand(
            () -> {
              X.setDouble(controller.getLeftX());
              Y.setDouble(controller.getLeftY());
              drive.driveCartesian(controller.getLeftX(), controller.getLeftY(),
                  controller.getRightX());
            }, drive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    Command command = tester.getSelectedCommand();
    return command;
  }
}
