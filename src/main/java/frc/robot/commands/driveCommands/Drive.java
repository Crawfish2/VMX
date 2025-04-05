package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TitanKilloughDrive;

/**
 * 直進するコマンド
 * 0で前進、90で右、-90で左に移動する。
 * 角度は度数法で指定する。
 */
public class Drive extends CommandBase {
  private final TitanKilloughDrive drive;
  private double speed;
  private double angle;

  /**
   * Creates a new ExampleCommand.
   *
   * @param angle The angle in degrees.
   */
  public Drive(double angle, TitanKilloughDrive drive) {
    this.angle = angle;
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    // ドライブにサブシステムを追加する。
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  // 初期化
  @Override
  public void initialize() {
    speed = 0.3;
  }

  // Called every time the scheduler runs while the command is scheduled.
  // コマンドが実行されるたびに呼び出される。
  @Override
  public void execute() {
    drive.drivePolar(speed, angle, 0);
  }

  // Called once the command ends or is interrupted.
  // コマンドが終了したか割り込まれたときに呼び出される。
  @Override
  public void end(boolean interrupted) {
    drive.stopMotor();
  }

  // Returns true when the command should end.
  // コマンドが終了するべきときにtrueを返す。
  @Override
  public boolean isFinished() {
    return false;
  }
}
