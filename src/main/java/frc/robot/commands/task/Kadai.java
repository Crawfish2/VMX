package frc.robot.commands.task;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Competitions;
import frc.robot.commands.Competitions.Direction;
import frc.robot.subsystems.SimpleCamera;
import frc.robot.subsystems.TitanKilloughDrive;
import frc.robot.subsystems.UltraSonicSensor;
import frc.robot.subsystems.SimpleCamera.ColorType;

public class Kadai {
  private final TitanKilloughDrive drive;
  private final SimpleCamera camera;
  private final UltraSonicSensor sonor;

  private final Competitions comp;

  public Kadai(TitanKilloughDrive drive, SimpleCamera camera, UltraSonicSensor sonor) {
    this.drive = drive;
    this.camera = camera;
    this.sonor = sonor;
    comp = new Competitions(drive, camera, sonor);

    final var tab = Shuffleboard.getTab("Kadai");
    tab.add(kadai1());
  }

  public CommandBase kadai1() {
    return new SequentialCommandGroup(
        camera.DetectColorCommand(),
        drive.odometry.ResetPoseCommand(new Pose2d(0, 600, Rotation2d.fromDegrees(90))),
        drive.RotateDistanceCommand(-90),
        drive.odometry.ResetPoseCommand(new Pose2d(0, 600, Rotation2d.fromDegrees(0))),

        comp.moveToPose(0, 600, 0),

        new ConditionalCommand(
            // 赤、青のとき
            new SequentialCommandGroup(
                comp.moveToPose(2.5 * 600, 600, 0),
                comp.PoseCollection(Direction.Right,
                    new Pose2d(2.5 * 600, 600, Rotation2d.fromDegrees(0))),
                comp.moveToPose(2.5 * 600, 0, 0),

                comp.moveToPose(3.5 * 600, 0, 0),
                new ConditionalCommand(
                    // 青のとき
                    new SequentialCommandGroup(
                        comp.moveToPose(3.5 * 600, 1 * 600, 0),
                        new WaitCommand(5)),
                    // 赤のとき
                    new WaitCommand(5),
                    () -> camera.getDetectedColor().equals(ColorType.BLUE)),

                // 戻る
                comp.moveToPose(3.5 * 600, 0, 0),
                comp.moveToPose(2.5 * 600, 0, 0),

                comp.moveToPose(2.5 * 600, 600, 0),
                comp.PoseCollection(Direction.Right,
                    new Pose2d(2.5 * 600, 600, Rotation2d.fromDegrees(0)))),


            // 黄のとき
            new SequentialCommandGroup(
                comp.moveToPose(1 * 600, 1 * 600, 0),
                comp.moveToPose(1 * 600, 2 * 600, 0),
                comp.moveToPose(3.5 * 600, 2 * 600, 0),

                new WaitCommand(5),
                // 戻る
                comp.moveToPose(1 * 600, 2 * 600, 0),
                comp.moveToPose(1 * 600, 1 * 600, 0)),

            () -> !camera.getDetectedColor().equals(ColorType.YELLOW)),

        comp.moveToPose(0, 600, 0));
  }
}
