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
import frc.robot.subsystems.NewElevator;
import frc.robot.subsystems.TitanKilloughDrive;

public class CommandTester {
  private final SendableChooser<Supplier<Command>> chooser;

  private final NetworkTableEntry speed;
  private final NetworkTableEntry distance;

  public CommandTester(TitanKilloughDrive drive, NewElevator elevator) {
    final ShuffleboardTab tab = Shuffleboard.getTab("CommandTester");

    speed = tab.add("Speed", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1.0, "max", 1.0))
        .getEntry();

    distance = tab.add("Distance", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 50.0))
        .getEntry();

    chooser = new SendableChooser<>();
    // 速度や角度はchooser.getSelected().get()が呼ばれた時点の値を使う
    chooser.setDefaultOption("DriveMotor",
        () -> new SequentialCommandGroup(drive.DriveCommand(speed.getDouble(0.0)).withTimeout(5)));

    chooser.addOption("RaiseElevator", () -> elevator.RaiseElevatorCommand(distance.getDouble(0)));

    chooser.addOption("LowerElevator", () -> elevator.LowerElevatorCommand(distance.getDouble(0)));
    tab.add(chooser);
  }

  public Command getSelectedCommand() {
    return chooser.getSelected().get();
  }
}
