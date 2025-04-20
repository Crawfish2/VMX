package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemExBase;
import frc.robot.commands.util.DeadlineCommand;
import static frc.robot.Constants.Ultrasonic.EchoPin;
import static frc.robot.Constants.Ultrasonic.PingPin;

/**
 * 超音波距離センサー
 */
public class UltraSonicSensor extends SubsystemExBase {
  private Ultrasonic sonar;

  public UltraSonicSensor() {
    sonar = new Ultrasonic(EchoPin, PingPin);
  }

  /**
   * 超音波センサーの距離を取得する
   * 実際の距離が測れる範囲にない場合、0を返す
   *
   * @return mm単位での距離、有効範囲外では、0を返す
   */
  public double getRangeMM() {
    return sonar.getRangeMM();
  }

  /**
   * 超音波距離センサーの読み取った距離が短くなると終了するデッドライン用コマンド
   *
   * @param deadline
   * @return
   */
  public Command SonicDeadlineCommand(double deadline) {
    return new DeadlineCommand(() -> sonar.getRangeMM() > deadline, this);
  }

  /**
   * 距離が有効な範囲にあるか調べる
   *
   * @return 範囲内にある時はtrue
   */
  public boolean isRangeValid() {
    return sonar.isRangeValid();
  }

  public void close() {
    sonar.close();
  }
}
