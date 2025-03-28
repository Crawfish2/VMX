package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * 回転するコマンド
 */
public class Rotate extends CommandBase {
  private static final ExampleSubsystem drive = RobotContainer.drive;
  private double speed;

  public Rotate(double speed) {
    // ドライブにサブシステムを追加する。
    this.speed = speed;
    addRequirements(drive);
  }

  // 初期化
  @Override
  public void initialize() {}

  // コマンドが実行されるたびに呼び出される。
  @Override
  public void execute() {
    drive.omniDrive.rotate(speed);
  }

  // コマンドが終了したか割り込まれたときに呼び出される。
  @Override
  public void end(boolean interrupted) {
    drive.omniDrive.move(0.0, 0.0);
  }

  // コマンドが終了するべきときにtrueを返す。
  @Override
  public boolean isFinished() {
    return false;
  }
}
