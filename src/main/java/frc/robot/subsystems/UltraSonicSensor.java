package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemExBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import static frc.robot.Constants.Ultrasonic.frontLeftEchoPin;
import static frc.robot.Constants.Ultrasonic.frontLeftPingPin;
import static frc.robot.Constants.Ultrasonic.frontRightEchoPin;
import static frc.robot.Constants.Ultrasonic.frontRightPingPin;
import static frc.robot.Constants.Ultrasonic.middleLeftEchoPin;
import static frc.robot.Constants.Ultrasonic.middleLeftPingPin;
import static frc.robot.Constants.Ultrasonic.middleRightEchoPin;
import static frc.robot.Constants.Ultrasonic.middleRightPingPin;

/**
 * 超音波距離センサー
 */
public class UltraSonicSensor extends SubsystemExBase {
  public static enum UltraSonicPosition {
    frontLeft(0), frontRight(1), middleLeft(2), middleRight(3);

    private final int index;

    private UltraSonicPosition(int index) {
      this.index = index;
    }
  }

  private final Ultrasonic[] sonars;
  private final Timer timer;

  public UltraSonicSensor() {
    sonars = new Ultrasonic[] {
        new Ultrasonic(frontLeftPingPin, frontLeftEchoPin),
        new Ultrasonic(frontRightPingPin, frontRightEchoPin),
        new Ultrasonic(middleLeftPingPin, middleLeftEchoPin),
        new Ultrasonic(middleRightPingPin, middleRightEchoPin)
    };

    timer = new Timer();
    timer.start();

    registerToShuffleboard();
  }

  /** Shuffleboardへの登録をする */
  private void registerToShuffleboard() {
    final var tab = Shuffleboard.getTab("UltraSonic");

    SendableRegistry.addChild(this, getSonar(UltraSonicPosition.frontLeft));
    SendableRegistry.addChild(this, getSonar(UltraSonicPosition.frontRight));
    SendableRegistry.addChild(this, getSonar(UltraSonicPosition.middleLeft));
    SendableRegistry.addChild(this, getSonar(UltraSonicPosition.middleRight));

    tab.add("FrontLeft", getSonar(UltraSonicPosition.frontLeft));
    tab.add("FrontRight", getSonar(UltraSonicPosition.frontRight));
    tab.add("MiddleLeft", getSonar(UltraSonicPosition.middleLeft));
    tab.add("MiddleRight", getSonar(UltraSonicPosition.middleRight));
  }

  private Ultrasonic getSonar(UltraSonicPosition pos) {
    return sonars[pos.index];
  }

  /**
   * 超音波センサーの距離を取得する
   * 実際の距離が測れる範囲にない場合、0を返す
   *
   * @return mm単位での距離、有効範囲外では、0を返す
   */
  public double getRangeMM(UltraSonicPosition pos) {
    return getSonar(pos).getRangeMM();
  }


  public double getForwardAvg() {
    return (getSonar(UltraSonicPosition.frontLeft).getRangeMM()
        + getSonar(UltraSonicPosition.frontRight).getRangeMM()) / 2;
  }

  /**
   * (前方左距離 - 前方右距離)
   */
  public double getForwardDiff() {
    return getSonar(UltraSonicPosition.frontLeft).getRangeMM()
        - getSonar(UltraSonicPosition.frontRight).getRangeMM();
  }

  /**
   * 超音波距離センサーの読み取った距離が短くなると終了するデッドライン用コマンド
   *
   * @param deadline
   * @return
   */
  public Command SonicDeadlineCommand(UltraSonicPosition pos, double deadline) {
    return new WaitUntilCommand(() -> getSonar(pos).getRangeMM() > deadline);
  }

  /**
   * 距離が有効な範囲にあるか調べる
   *
   * @return 範囲内にある時はtrue
   */
  public boolean isRangeValid(UltraSonicPosition pos) {
    return getSonar(pos).isRangeValid();
  }

  @Override
  public void periodic() {
    if (timer.advanceIfElapsed(0.040)) {
      for (Ultrasonic sonar : sonars) {
        sonar.ping();
      }
    }
  }
}
