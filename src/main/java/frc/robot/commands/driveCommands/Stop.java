package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * 停止するコマンド
 */
public class Stop extends CommandBase {
  private static final ExampleSubsystem drive = RobotContainer.drive;

  public Stop() {
    addRequirements(drive);
  }

  // 初期化
  @Override
  public void initialize() {
    drive.omniDrive.stopMotor();
  }

  // コマンドが実行されるたびに呼び出される。
  @Override
  public void execute() {
    drive.omniDrive.stopMotor();
  }

  @Override
  public void end(boolean interrupted) {
    drive.omniDrive.stopMotor();
  }

  // コマンドが終了するべきときにtrueを返す。
  @Override
  public boolean isFinished() {
    return false;
  }
}
