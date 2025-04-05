package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TitanKilloughDrive;

/**
 * 回転するコマンド
 * -1で右回転、1で左回転する。
 * 角度は度数法で指定する。
 */
public class Rotate extends CommandBase {
  private final TitanKilloughDrive drive;
  private double speed;

  public Rotate(double speed, TitanKilloughDrive drive) {
    // ドライブにサブシステムを追加する。
    this.speed = speed;
    this.drive = drive;
    addRequirements(drive);
  }

  // 初期化
  @Override
  public void initialize() {}

  // コマンドが実行されるたびに呼び出される。
  @Override
  public void execute() {
    drive.driveCartesian(0, 0, speed);
  }

  // コマンドが終了したか割り込まれたときに呼び出される。
  @Override
  public void end(boolean interrupted) {
    drive.stopMotor();
  }

  // コマンドが終了するべきときにtrueを返す。
  @Override
  public boolean isFinished() {
    return false;
  }
}
