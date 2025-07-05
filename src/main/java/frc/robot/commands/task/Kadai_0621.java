package frc.robot.commands.task;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Competitions;
import frc.robot.commands.Competitions.Direction;
import frc.robot.subsystems.SimpleCamera;
import frc.robot.subsystems.TitanKilloughDrive;
import frc.robot.subsystems.UltraSonicSensor;
import frc.robot.subsystems.SimpleCamera.ColorType;
import frc.robot.util.Box;
import frc.robot.util.SendableBox;

public class Kadai_0621 {
  private final TitanKilloughDrive drive;
  private final UltraSonicSensor sonar;
  private final SimpleCamera camera;

  private final Competitions comp;

  public Kadai_0621(TitanKilloughDrive drive, SimpleCamera camera, UltraSonicSensor sonar) {
    this.drive = drive;
    this.camera = camera;
    this.sonar = sonar;
    comp = new Competitions(drive, camera, sonar);

    final var tab = Shuffleboard.getTab("Kadai");
    tab.add(Commands.withName("basicMove", basicMove()));
  }

  private Pose2d pose(double x, double y, double deg) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(deg));
  }

  private CommandBase getColor(Box<ColorType> resultOutput) {
    return camera.DetectColorCommand()
        .andThen(Commands.runOnce(() -> resultOutput.set(camera.getDetectedColor())));
  }

  public CommandBase basicMove() {
    final var tab = Shuffleboard.getTab("Kadai");

    // 正面向きで開始
    Supplier<CommandBase> move1 =
        () -> Commands.withName("BasicMove1",
            new SequentialCommandGroup(
                drive.odometry.ResetPoseCommand(pose(0, 0, 0)),
                comp.moveForwardDistanceSensor(
                    pose(2.5 * 600, 0, 0),
                    Direction.Left),
                comp.moveToPose(2.5 * 600, 2 * 600, 0),
                comp.moveForwardDistanceSensor(
                    pose(3.5 * 600, 2 * 600, 0),
                    Direction.Right),
                comp.PoseCollection(Direction.Right,
                    pose(3.5 * 600, 2 * 600, 0)),
                new WaitCommand(3),
                // 帰る
                comp.moveForwardDistanceSensor(
                    pose(2.5 * 600, 2 * 600, 0),
                    Direction.Right),
                comp.moveToPose(2.5 * 600, 0, 0),
                comp.moveForwardDistanceSensor(
                    pose(0, 0, 0),
                    Direction.Left),
                comp.moveToPose(0, 0, 90),
                comp.PoseCollection(Direction.Right, pose(0, 0, 90))
            //
            ));

    final SendableBox<ColorType> alphaPack = new SendableBox<>(ColorType.PREPARING);
    final SendableBox<ColorType> betaPack = new SendableBox<>(ColorType.PREPARING);

    tab.add("alphaPack", alphaPack);
    tab.add("betaPack", betaPack);

    // 正面向きで開始
    Supplier<CommandBase> move2 =
        () -> Commands.withName("BasicMove2",
            new SequentialCommandGroup(
                Commands.runOnce(() -> {
                  alphaPack.set(ColorType.PREPARING);
                  betaPack.set(ColorType.PREPARING);
                }),

                // スタートしてゴール
                drive.odometry.ResetPoseCommand(pose(0, 0, 0)),
                comp.moveForwardDistanceSensor(pose(600, 0, 0), Direction.Left),
                comp.moveForwardDistanceSensor(pose(0, 0, 0), Direction.Left),

                comp.PoseCollection(Direction.Right,
                    new Pose2d(0, 0, Rotation2d.fromDegrees(90))),
                comp.moveToPose(1.75 * 600, 0, 90),
                comp.moveToPose(1.75 * 600, 1 * 600, 90),
                getColor(alphaPack),
                new ConditionalCommand(new SequentialCommandGroup(
                    comp.moveToPose(1.75 * 600, 2 * 600, 90),
                    comp.moveToPose(1.75 * 600, 0, 90)//
                ), new SequentialCommandGroup(
                    comp.moveToPose(0, 600, 90),
                    comp.moveToPose(0, 2 * 600, 90),
                    comp.moveToPose(1.75 * 600, 600, 90)//
                ), () -> ColorType.BLUE.equals(alphaPack.get())),
                comp.moveToPose(1.75, 0, 90),
                comp.moveToPose(0, 0, 90),
                comp.PoseCollection(Direction.Right, pose(0, 0, 90))
            //
            ));

    tab.add(move1.get());
    tab.add(move2.get());

    return move1.get().andThen(move2.get());
  }
}
