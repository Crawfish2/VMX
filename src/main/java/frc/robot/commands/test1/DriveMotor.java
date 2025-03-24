package frc.robot.commands.test1;

import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.driveCommands.Drive;

public class DriveMotor extends AutoCommand {
    // コンストラクタ
    public DriveMotor(double angle) {
        // Driveクラスを呼び出し、angleを引数に渡す
        super(new Drive(angle).withTimeout(5));
    }

    // ロボットを直進させるメソッド
    public void driveStraight(double speed) {
        // 直進するために角度を0に設定
        new Drive(0).withTimeout(5).schedule();
        
    }
}