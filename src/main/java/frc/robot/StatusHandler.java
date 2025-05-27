package frc.robot;

import com.studica.frc.MockDS;
import edu.wpi.first.wpilibj.AnalogInput;


/**
 * 競技用のロボットの開始、停止スイッチなどを通して、
 * Driver StationのEnable, Disableの状態を変える
 * ステータスランプの表示も持つ？
 */
public class StatusHandler {
  // Driver Station
  private final MockDS ds;

  private final AnalogInput startButton;
  private final AnalogInput resetButton;
  private final AnalogInput stopButton;
  private final AnalogInput bumberSensor;

  // Analog Input 0-3
  private final int startButtonChannel = 0;
  private final int resetButtonChannel = 1;
  private final int stopButtonChannel = 2;
  private final int bumberSensorChannel = 3;

  private boolean isIdle = false;

  public StatusHandler(MockDS ds) {
    this.ds = ds;
    startButton = new AnalogInput(startButtonChannel);
    resetButton = new AnalogInput(resetButtonChannel);
    stopButton = new AnalogInput(stopButtonChannel);
    bumberSensor = new AnalogInput(bumberSensorChannel);
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
    } else if ((getInput(stopButton) || getInput(bumberSensor)) && isEnabled) {
      ds.disable();
      isIdle = false;
    }
  }
}
