package frc.robot.commands.test1;
// AutoCommandクラスをインポート
import frc.robot.commands.auto.AutoCommand;
// Driveクラスをインポート
import frc.robot.commands.driveCommands.Drive;

public class DriveMotor extends AutoCommand {
    // コンストラクタ
    public DriveMotor(double angle) {
        // Driveクラスを呼び出し、angleを引数に渡す
        super(new Drive(angle).withTimeout(5));
    }
}