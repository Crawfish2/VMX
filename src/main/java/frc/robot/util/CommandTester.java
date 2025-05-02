package frc.robot.util;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.task.DriveCorners;
import frc.robot.commands.task.PubKadai1;
import frc.robot.commands.test.DriveTri;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SimpleCamera;
import frc.robot.subsystems.TitanKilloughDrive;
import frc.robot.subsystems.UltraSonicSensor;

public class CommandTester {
  private final SendableChooser<Supplier<Command>> chooser;

  private final NetworkTableEntry speed;
  private final NetworkTableEntry angle;
  private final NetworkTableEntry distance;

  public CommandTester(TitanKilloughDrive drive, UltraSonicSensor sonar, SimpleCamera camera,
      Elevator elevator) {
    final ShuffleboardTab tab = Shuffleboard.getTab("CommandTester");

    speed = tab.add("Speed", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1.0, "max", 1.0))
        .getEntry();

    angle = tab.add("Angle", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -360.0, "max", 360.0))
        .getEntry();

    distance = tab.add("Distance", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 50.0))
        .getEntry();

    chooser = new SendableChooser<>();
    // 速度や角度はchooser.getSelected().get()が呼ばれた時点の値を使う
    chooser.setDefaultOption("DriveMotor",
        () -> new SequentialCommandGroup(drive.DriveCommand(speed.getDouble(0.0)).withTimeout(5)));

    chooser.addOption("Rotate",
        () -> drive.RotateDistanceCommand(angle.getDouble(0.0)).withTimeout(5));
    chooser.addOption("DriveCorners", () -> new DriveCorners(drive));
    chooser.addOption("DriveTri", () -> new DriveTri(drive));

    chooser.addOption("DriveDistance",
        () -> drive.DriveDistanceCommand(angle.getDouble(0), distance.getDouble(0)));

    chooser.addOption("ResetElevatorEncoder", () -> elevator.ResetElevatorEncoderCommand());
    chooser.addOption("RaiseElevator", () -> elevator.RaiseElevatorCommand(distance.getDouble(0)));

    chooser.addOption("LowerElevator", () -> elevator.LowerElevatorCommand(distance.getDouble(0)));

    chooser.addOption("Public Kadai 1", () -> new PubKadai1(drive, camera));
    tab.add(chooser);
  }

  public Command getSelectedCommand() {
    return chooser.getSelected().get();
  }
}
