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
import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SimpleCamera;
import frc.robot.subsystems.TitanKilloughDrive;
import frc.robot.subsystems.UltraSonicSensor;
import frc.robot.util.CommandTester;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
  private final Elevator elevator = new Elevator();
  // private final Arm arm = new Arm();

  private final CommandTester tester = new CommandTester(drive, sonar, camera, elevator);

  private final PS4Controller controller = new PS4Controller(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();


    // 左スティック X,Y: 水平移動
    // 右スティック X: 回転
    // R1, R2: エレベーター下降、上昇
    // L1, L2: アーム閉じる、開く (予定)
    // Set default commands
    drive.setDefaultCommand(
        Commands.run(
            () -> {
              drive.driveCartesian(controller.getLeftX() * 0.5, -controller.getLeftY() * 0.5,
                  controller.getRightX() * 0.5);
            }, drive));

    elevator.setDefaultCommand(Commands.run(() -> {
      if (controller.getR1Button()) {
        elevator.lower();
      } else if (controller.getR2Button()) {
        elevator.raise();
      } else {
        elevator.stopMotor();
      }
    }, elevator));

    // arm.setDefaultCommand(Commands.run(() -> {
    // if (controller.getL1Button()) {
    // arm.grab();
    // } else if (controller.getL2Button()) {
    // arm.release();
    // } else {
    // arm.stopMotor();
    // }
    // }, arm));
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
