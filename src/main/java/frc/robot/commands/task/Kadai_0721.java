package frc.robot.commands.task;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Competitions;
import frc.robot.commands.Competitions.Direction;
import frc.robot.subsystems.SimpleCamera;
import frc.robot.subsystems.TitanKilloughDrive;
import frc.robot.subsystems.UltraSonicSensor;
import frc.robot.subsystems.SimpleCamera.ColorType;
import frc.robot.util.Box;
import frc.robot.util.SendableBox;

public class Kadai_0721 {
  private final SimpleCamera camera;

  private final Competitions comp;

  public Kadai_0721(TitanKilloughDrive drive, SimpleCamera camera, UltraSonicSensor sonar) {
    this.camera = camera;
    comp = new Competitions(drive, camera, sonar);

    final var tab = Shuffleboard.getTab("Kadai");
    tab.add(Commands.withName("kadai1", kadai1()));
  }

  private Pose2d pose(double x, double y, double deg) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(deg));
  }

  private CommandBase getColor(Box<ColorType> resultOutput) {
    return camera.DetectColorCommand()
        .andThen(Commands.runOnce(() -> resultOutput.set(camera.getDetectedColor())));
  }

  private CommandBase kadai1() {
    final var tab = Shuffleboard.getTab("Kadai");

    // 右向きでスタート
    final Supplier<CommandBase> start = () -> Commands.withName("move1", new SequentialCommandGroup(
        comp.PoseCollection(Direction.Right, pose(0, 0, 90)),
        comp.moveToPose(0, 0, 0),
        comp.moveForwardDistanceSensor(pose(1.75 * 600, 0, 0), Direction.Left),
        comp.moveToPose(1.75 * 600, 0, 90),
        comp.moveToPose(1.75 * 600, 1 * 600, 90)
    //
    ));

    final SendableBox<ColorType> currentColor = new SendableBox<>(ColorType.PREPARING);
    final SendableDoubleBox colorCenterAreaX = new SendableDoubleBox(0.0);
    final SendableDoubleBox colorCenterAreaY = new SendableDoubleBox(0.0);

    tab.add("currentColor", currentColor);
    tab.add("colorCenterAreaX", colorCenterAreaX);
    tab.add("colorCenterAreaY", colorCenterAreaY);

    final Supplier<CommandBase> catchLineA = () -> Commands.withName("catchLineA",
        new SequentialCommandGroup(
            comp.moveToPose(2 * 600, 1 * 600, 90),
            getColor(currentColor),
            Commands.runOnce(() -> currentColor.set(camera.getDetectedColor())),
            Commands.runOnce(() -> colorCenterAreaX.set(camera.getDetectedCenterAreaX())),
            Commands.runOnce(() -> colorCenterAreaY.set(camera.getDetectedCenterAreaY()))
        //
        ));


    final Supplier<CommandBase> startGoal = () -> Commands.withName("startGoal",
        new SequentialCommandGroup(
            comp.PoseCollection(Direction.Right, pose(0, 0, 90)),
            comp.moveToPose(0, 0, 90),
            comp.moveToPose(0, 0, 0),
            comp.moveForwardDistanceSensor(pose(1 * 600, 0, 0), Direction.Left),
            comp.moveForwardDistanceSensor(pose(0, 0, 0), Direction.Left),
            comp.moveToPose(0, 0, 90),
            comp.PoseCollection(Direction.Right, pose(0, 0, 90))

        //
        ));

    tab.add(start.get());
    tab.add(catchLineA.get());
    tab.add(startGoal.get());

    return new SequentialCommandGroup(start.get(), catchLineA.get());
  }
}


class SendableDoubleBox extends Box<Double> implements Sendable {
  public SendableDoubleBox(double value) {
    super(value);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("value", this::get, this::set);
  }
}
