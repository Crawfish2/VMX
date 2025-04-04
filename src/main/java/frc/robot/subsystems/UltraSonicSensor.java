package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.UltrasonicEcho;
import static frc.robot.Constants.UltrasonicPing;

/**
 * 超音波距離センサー
 */
public class UltraSonicSensor extends SubsystemBase {
  private Ultrasonic sonar;

  public UltraSonicSensor() {
    sonar = new Ultrasonic(UltrasonicPing, UltrasonicEcho);
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
