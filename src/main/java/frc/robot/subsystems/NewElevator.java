package frc.robot.subsystems;

import com.studica.frc.ServoContinuous;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemExBase;
import frc.robot.util.SaferMotor;
import static frc.robot.Constants.TitanConstants.ElevatorConstants.safeLimitSwitchPin;;

/** アームのエレベーター */
// TODO: エレベーターのテストをする
public class NewElevator extends SubsystemExBase {
  private final ServoContinuous servo;
  private final SaferMotor drive;

  /** 下側のリミットスイッチ */
  private final AnalogInput safeLimitSwitch;

  private static final double SPEED = 0.3;

  private static final int ELEVATOR_CHANNEL = 3;

  public NewElevator() {
    servo = new ServoContinuous(ELEVATOR_CHANNEL);
    drive = new SaferMotor(servo);
    safeLimitSwitch = new AnalogInput(safeLimitSwitchPin);

    registerToShuffleboard();
  }

  private boolean getInput(final AnalogInput input) {
    return safeLimitSwitch.getValue() >= 4000;
  }

  /** Shuffleboardへの登録をする */
  private void registerToShuffleboard() {
    SendableRegistry.addChild(this, drive);

    Shuffleboard.getTab("Elevator").add(this);
  }

  /** エレベーターを上げる */
  public void raise() {
    drive.drive(SPEED);
  }

  /** エレベーターを下げる */
  public void lower() {
    drive.drive(-SPEED);
  }

  /** エレベーターのモーターを停止する */
  public void stopMotor() {
    drive.stopMotor();
  }

  /** エレベーターを下げる、リミットスイッチに接触すると止まる */
  public void lowerSafe() {
    // 接触していない
    if (getInput(safeLimitSwitch)) {
      lower();
    }
  }

  /**
   * 上昇するコマンド
   */
  public CommandBase RaiseCommand() {
    return run(this::raise);
  }

  /**
   * 下降するコマンド
   */
  public CommandBase LowerCommand() {
    return run(this::lowerSafe); // 安全のため
  }

  public CommandBase StopCommand() {
    return runOnce(this::stopMotor);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Motor Speed", servo::get, drive::drive);
  }
}
