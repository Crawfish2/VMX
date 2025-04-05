package frc.robot.commands.sensors;

import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.util.DeadlineCommand;
import frc.robot.subsystems.UltraSonicSensor;

/**
 * 超音波距離センサーの読み取った距離が短くなると終了する
 */
public class SonicSensorDeadline extends AutoCommand {
  public SonicSensorDeadline(double deadline, UltraSonicSensor sonar) {
    super(new DeadlineCommand(() -> sonar.getRangeMM() > deadline, sonar));
  }
}
