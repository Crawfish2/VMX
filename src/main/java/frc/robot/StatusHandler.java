package frc.robot;

import com.studica.frc.MockDS;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;


/**
 * 競技用のロボットの開始、停止スイッチなどを通して、
 * Driver StationのEnable, Disableの状態を変える
 * ステータスランプの表示も持つ？
 */
public class StatusHandler implements Sendable {
  // Driver Station
  private final MockDS ds;

  private final AnalogInput startButton;
  private final AnalogInput resetButton;
  private final AnalogInput stopButton;
  private final AnalogInput bumperSensor;

  // Analog Input 0-3
  private final int startButtonChannel = 0;
  private final int resetButtonChannel = 1;
  private final int stopButtonChannel = 2;
  private final int bumperSensorChannel = 3;

  private boolean isIdle = false;

  public StatusHandler(MockDS ds) {
    this.ds = ds;
    startButton = new AnalogInput(startButtonChannel);
    resetButton = new AnalogInput(resetButtonChannel);
    stopButton = new AnalogInput(stopButtonChannel);
    bumperSensor = new AnalogInput(bumperSensorChannel);

    registerToShuffleboard();
  }

  private void registerToShuffleboard() {
    final ShuffleboardTab tab = Shuffleboard.getTab("StatusHandler");
    tab.add("StatusHandler", this);

    SendableRegistry.addChild(this, startButton);
    SendableRegistry.addChild(this, resetButton);
    SendableRegistry.addChild(this, stopButton);
    SendableRegistry.addChild(this, bumperSensor);
  }

  private boolean getInput(final AnalogInput input) {
    return input.getValue() >= 4000;
  }

  public void updateDS(boolean isEnabled) {
    if (getInput(startButton) && isIdle && !isEnabled) {
      ds.enable();
      isIdle = false;
    } else if (getInput(resetButton)) {
      if (isEnabled) {
        ds.disable();
      }
      isIdle = true;
    } else if ((getInput(stopButton) || getInput(bumperSensor)) && isEnabled) {
      ds.disable();
      isIdle = false;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("startButton", startButton::getValue, null);
    builder.addDoubleProperty("stopButton", stopButton::getValue, null);
    builder.addDoubleProperty("resetButton", resetButton::getValue, null);
    builder.addDoubleProperty("bumperSensor", bumperSensor::getValue, null);
  }
}
