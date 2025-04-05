package frc.robot.util;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.DriveMotor;
// import frc.robot.commands.driveCommands.Drive;
import frc.robot.commands.driveCommands.DriveDistance;
import frc.robot.commands.driveCommands.Rotate;
import frc.robot.commands.driveCommands.Stop;
// import frc.robot.commands.sensors.SonicSensorDeadline;
import frc.robot.commands.task.DriveCorners;
import frc.robot.commands.test.DriveTri;

public class CommandTester {
  private final SendableChooser<Supplier<Command>> chooser;

  private final NetworkTableEntry speed;
  private final NetworkTableEntry angle;
  private final NetworkTableEntry distance;

  public CommandTester() {
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
        () -> new SequentialCommandGroup(new DriveMotor(speed.getDouble(0.0)),
            new Stop().withTimeout(1)));
    chooser.addOption("Rotate", () -> new Rotate(speed.getDouble(0.0))
        .withTimeout(5)
        .andThen(new Stop().withTimeout(1)));
    chooser.addOption("DriveCorners", () -> new DriveCorners());
    chooser.addOption("DriveTri", () -> new DriveTri());
    // chooser.addOption("SonicSensor",
    // () -> new ParallelDeadlineGroup(new SonicSensorDeadline(distance.getDouble(0), null),
    // new Drive(angle.getDouble(0.0))));
    chooser.addOption("DriveDistance",
        () -> new DriveDistance(angle.getDouble(0), distance.getDouble(0)));
    tab.add(chooser);
  }

  public Command getSelectedCommand() {
    return chooser.getSelected().get();
  }
}
