package frc.robot.commands.task;

import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
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

public class Kadai_0503 {
  private final TitanKilloughDrive drive;
  private final SimpleCamera camera;
  private final UltraSonicSensor sonar;

  private final Competitions comp;

  public Kadai_0503(TitanKilloughDrive drive, SimpleCamera camera, UltraSonicSensor sonar) {
    this.drive = drive;
    this.camera = camera;
    this.sonar = sonar;
    comp = new Competitions(drive, camera, sonar);

    final var tab = Shuffleboard.getTab("Kadai");
    tab.add(Commands.withName("kadai1", kadai1()));
    tab.add(Commands.withName("kadai2", kadai2()));
  }

  public CommandBase kadai1() {
    return new SequentialCommandGroup(
        camera.DetectColorCommand(),
        drive.odometry.ResetPoseCommand(
            new Pose2d(0, 600, Rotation2d.fromDegrees(90))),
        comp.moveToPose(0, 600, 0),

        new ConditionalCommand(
            // 赤、青のとき
            new SequentialCommandGroup(
                comp.moveToPose(2.5 * 600, 600, 0),
                comp.PoseCollection(Direction.Right,
                    new Pose2d(2.5 * 600,
                        600,
                        Rotation2d.fromDegrees(
                            0))),
                comp.moveToPose(2.5 * 600, 0, 0),

                comp.moveToPose(3.5 * 600, 0, 0),
                new ConditionalCommand(
                    // 青のとき
                    new SequentialCommandGroup(
                        comp.moveToPose(3.5
                            * 600,
                            1 * 600,
                            0),
                        new WaitCommand(5)),
                    // 赤のとき
                    new WaitCommand(5),
                    () -> camera.getDetectedColor()
                        .equals(ColorType.BLUE)),

                // 戻る
                comp.moveToPose(3.5 * 600, 0, 0),
                comp.moveToPose(2.5 * 600, 0, 0),

                comp.moveToPose(2.5 * 600, 600, 0),
                comp.PoseCollection(Direction.Right,
                    new Pose2d(2.5 * 600,
                        600,
                        Rotation2d.fromDegrees(
                            0)))),


            // 黄のとき
            new SequentialCommandGroup(
                comp.moveToPose(1 * 600, 1 * 600,
                    0),
                comp.moveToPose(1 * 600, 2 * 600,
                    0),
                comp.moveForwardDistanceSensor(
                    new Pose2d(3.5 * 600,
                        2 * 600,
                        Rotation2d.fromDegrees(
                            0)),
                    Direction.Right)
                    .withTimeout(14),
                comp.PoseCollection(Direction.Right,
                    new Pose2d(3.5 * 600,
                        2 * 600,
                        Rotation2d.fromDegrees(
                            0))),
                // comp.moveToPose(3.5 * 600, 2 *
                // 600, 0),

                new WaitCommand(5),
                // 戻る
                comp.moveForwardDistanceSensor(
                    new Pose2d(1 * 600,
                        2 * 600,
                        Rotation2d.fromDegrees(
                            0)),
                    Direction.Right)
                    .withTimeout(7),
                comp.moveToPose(1 * 600, 1 * 600,
                    0)),

            () -> !camera.getDetectedColor()
                .equals(ColorType.YELLOW)),

        comp.moveToPose(0, 600, 0));
  }

  public Sendable createSendable(String name, Supplier<NetworkTableValue> getter,
      Consumer<NetworkTableValue> setter) {
    final var sendable = new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.addValueProperty("value", getter, setter);
      }
    };
    SendableRegistry.setName(sendable, name);
    return sendable;
  }

  public CommandBase kadai2() {
    // 初期の向き: 右
    final Box<ColorType> areaColorA = new Box<ColorType>(ColorType.PREPARING); // 右側
    final Box<ColorType> areaColorB = new Box<ColorType>(ColorType.PREPARING); // 左側

    final var tab = Shuffleboard.getTab("Kadai");

    BiFunction<String, Box<ColorType>, Void> sendColorType = (name, colorType) -> {
      tab.add(name,
          createSendable(name,
              () -> NetworkTableValue.makeString(
                  colorType.get().toString()),
              (value) -> colorType.set(ColorType
                  .valueOf(value.getString()))));
      return null;
    };

    sendColorType.apply("areaColorA", areaColorA);
    sendColorType.apply("areaColorB", areaColorB);

    final Supplier<CommandBase> readIroshiji =
        () -> Commands.withName("readIroshiji", new SequentialCommandGroup(
            Commands.runOnce(() -> {
              areaColorA.set(ColorType.PREPARING);
              areaColorB.set(ColorType.PREPARING);
            }, camera),

            comp.PoseCollection(Direction.Right,
                new Pose2d(0, 1 * 600,
                    Rotation2d.fromDegrees(
                        90))),
            // drive.odometry.ResetPoseCommand(new Pose2d(0,
            // 600,
            // Rotation2d.fromDegrees(90))),

            // 色指示板A
            camera.DetectColorCommand(),
            Commands.runOnce(
                () -> areaColorA.set(
                    camera.getDetectedColor()),
                camera),

            comp.moveToPose(600, 600 - 50, 90),
            comp.moveToPose(600, 600 - 50, -90),
            comp.moveToPose(0, 600 - 50, -90),
            comp.PoseCollection(Direction.Left,
                new Pose2d(0, 600,
                    Rotation2d.fromDegrees(
                        -90))),

            // 色指示板B
            camera.DetectColorCommand(),
            Commands.runOnce(
                () -> areaColorB.set(
                    camera.getDetectedColor()),
                camera)

        // 左向きで終了
        //
        ));


    final Box<ColorType> alphaPack =
        new Box<SimpleCamera.ColorType>(ColorType.PREPARING);
    final Box<ColorType> betaPack =
        new Box<SimpleCamera.ColorType>(ColorType.PREPARING);
    final Box<ColorType> gammaPack =
        new Box<SimpleCamera.ColorType>(ColorType.PREPARING);

    sendColorType.apply("alphaPack", alphaPack);
    sendColorType.apply("betaPack", betaPack);
    sendColorType.apply("gammaPack", gammaPack);

    // パックを運べる状態にする
    final BiFunction<Box<ColorType>, Double, CommandBase> capturePack =
        (colorPack, targetX) -> Commands.withName("capturePack",
            new SequentialCommandGroup(
                // 左向きで開始
                comp.moveToPose(2.5 * 600, 600,
                    -90),

                comp.moveToPose(targetX, 600, -90),
                camera.DetectColorCommand(),
                Commands.runOnce(
                    () -> colorPack.set(
                        camera.getDetectedColor()),
                    camera),

                // 回収
                comp.moveToPose(targetX, 0, -90),
                comp.CollectForward(new Pose2d(
                    targetX, 0,
                    Rotation2d.fromDegrees(
                        -90))),
                comp.moveToPose(targetX, 0, 0),
                // (2.5 * 600 + 50, 0, 0)で終わる
                comp.moveForwardDistanceSensor(
                    new Pose2d(2.5 * 600
                        + 50,
                        0,
                        Rotation2d.fromDegrees(
                            0)),
                    Direction.Left)
            // comp.moveToPose(2.5 * 600 + 50, 0, 0)
            ));


    // パックを指定回収エリアに運ぶ
    // パックを回収した状態から開始
    // 回収エリアから出て終了
    final Function<Box<ColorType>, CommandBase> carryPack =
        (packColor) -> Commands.withName("carryPack",
            new SequentialCommandGroup(new ConditionalCommand(
                new SequentialCommandGroup(
                    // areaAのとき
                    comp.moveToPose(2.5
                        * 600
                        + 50,
                        0,
                        0),
                    comp.moveForwardDistanceSensor(
                        new Pose2d(3.5 * 600,
                            0,
                            Rotation2d.fromDegrees(
                                0)),
                        Direction.Left),
                    // comp.moveToPose(3.5
                    // *
                    // 600, 0, 0),
                    comp.PoseCollection(
                        Direction.Right,
                        new Pose2d(3.5 * 600,
                            0,
                            Rotation2d.fromDegrees(
                                0))),
                    comp.moveToPose(2.5
                        * 600,
                        0,
                        0)),
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        // areaBのとき
                        comp.moveToPose(2.5
                            * 600
                            + 50,
                            600 + 50,
                            0),
                        comp.moveToPose(3.5
                            * 600,
                            600 + 50,
                            0),
                        comp.PoseCollection(
                            Direction.Right,
                            new Pose2d(3.5 * 600,
                                600,
                                Rotation2d.fromDegrees(
                                    0))),
                        comp.moveToPose(2.5
                            * 600,
                            600,
                            0)),
                    new SequentialCommandGroup(
                        // 回収エリアのとき
                        comp.moveToPose(2.5
                            * 600
                            + 50,
                            2 * 600,
                            0),
                        comp.moveForwardDistanceSensor(
                            new Pose2d(3.5 * 600,
                                2 * 600,
                                Rotation2d.fromDegrees(
                                    0)),
                            Direction.Right),
                        // comp.moveToPose(3.5
                        // *
                        // 600,
                        // 2
                        // *
                        // 600,
                        // 0),
                        comp.PoseCollection(
                            Direction.Right,
                            new Pose2d(3.5 * 600,
                                2 * 600,
                                Rotation2d.fromDegrees(
                                    0))),
                        comp.moveToPose(2.5
                            * 600,
                            2 * 600,
                            0)),
                    () -> packColor.equals(
                        areaColorB)),
                () -> packColor.equals(areaColorA)),
                comp.moveToPose(2.5 * 600, 600,
                    0)));

    tab.add(readIroshiji.get());

    tab.add("Capture&Carry GammaPack",
        new SequentialCommandGroup(capturePack.apply(gammaPack, 2.5 * 600),
            carryPack.apply(gammaPack)));
    tab.add("Capture&Carry BetaPack",
        new SequentialCommandGroup(capturePack.apply(betaPack, 1.75 * 600),
            carryPack.apply(betaPack)));
    tab.add("Capture&Carry AlphaPack",
        new SequentialCommandGroup(capturePack.apply(alphaPack, 1.0 * 600),
            carryPack.apply(alphaPack)));

    // ゴール(スタート)に戻る
    final Supplier<CommandBase> gotoGoal = () -> Commands.withName("Goto Goal",
        new SequentialCommandGroup(
            comp.moveToPose(2.5 * 600, 600 - 50, -90),
            comp.moveToPose(0, 600 - 50, -90),
            comp.PoseCollection(Direction.Left,
                new Pose2d(0, 600 - 50, Rotation2d
                    .fromDegrees(-90)))
        //
        ));

    tab.add(gotoGoal.get());


    // SequentialCommandGroupに追加した内部コマンドを個別にスケジュールすることはできない
    // コマンド作成を関数化することで解決する
    return new SequentialCommandGroup(
        readIroshiji.get(),
        capturePack.apply(gammaPack, 2.5 * 600),
        carryPack.apply(gammaPack),

        capturePack.apply(betaPack, 1.75 * 600),
        carryPack.apply(betaPack),

        capturePack.apply(alphaPack, 1.0 * 600),
        carryPack.apply(alphaPack),

        gotoGoal.get()
    //
    );
  }
}
