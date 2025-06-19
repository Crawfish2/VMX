package frc.robot;

import com.studica.frc.MockDS;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static frc.robot.Constants.StatusHandlerConstants.bumperSensorChannel;
import static frc.robot.Constants.StatusHandlerConstants.idleSignalChannel;
import static frc.robot.Constants.StatusHandlerConstants.resetButtonChannel;
import static frc.robot.Constants.StatusHandlerConstants.runningSignalChannel;
import static frc.robot.Constants.StatusHandlerConstants.startButtonChannel;
import javax.annotation.Nullable;
import static frc.robot.Constants.StatusHandlerConstants.emergencyButtonChannel;

/**
 * 競技用のロボットの開始、停止スイッチなどを通して、
 * Driver StationのEnable, Disableの状態を変える
 * ステータスランプの表示も持つ？
 */
public class StatusHandler implements Sendable {
  // Driver Station
  private final MockDS ds;

  /** getInput(startButton) == trueのとき、ボタンが押されている */
  private final AnalogInput startButton;
  /** getInput(startButton) == trueのとき、ボタンが押されている */
  private final AnalogInput resetButton;
  /** getInput(startButton) == falseのとき、ボタンが押されている */
  private final AnalogInput emergencyButton;

  private final AnalogInput bumperSensor;

  private final DigitalOutput idleSignal;
  private final DigitalOutput runningSignal;

  private @Nullable CommandBase task;

  public StatusHandler(MockDS ds) {
    this.ds = ds;
    startButton = new AnalogInput(startButtonChannel);
    resetButton = new AnalogInput(resetButtonChannel);
    emergencyButton = new AnalogInput(emergencyButtonChannel);
    bumperSensor = new AnalogInput(bumperSensorChannel);

    idleSignal = new DigitalOutput(idleSignalChannel);
    runningSignal = new DigitalOutput(runningSignalChannel);

    registerToShuffleboard();
  }

  private void registerToShuffleboard() {
    final ShuffleboardTab tab = Shuffleboard.getTab("StatusHandler");
    tab.add("StatusHandler", this);

    SendableRegistry.addChild(this, startButton);
    SendableRegistry.addChild(this, resetButton);
    SendableRegistry.addChild(this, emergencyButton);
    SendableRegistry.addChild(this, bumperSensor);
  }

  private boolean getInput(final AnalogInput input) {
    return input.getValue() >= 4000;
  }

  public void setTask(CommandBase task) {
    this.task = task;
  }

  private boolean isScheduled() {
    return task != null && CommandScheduler.getInstance().isScheduled(task);
  }

  private void schedule() {
    if (!isScheduled()) {
      task.schedule();
    }
  }

  public void updateDS(boolean isEnabled) {
    if (getInput(startButton) && isEnabled) {
      schedule();
    } else if (getInput(resetButton)) {
      CommandScheduler.getInstance().cancelAll();
      if (isEnabled) {
        ds.enable();
      }
    } else if (!getInput(emergencyButton) || getInput(bumperSensor)) {
      CommandScheduler.getInstance().cancelAll();
      if (!isEnabled) {
        ds.disable();
      }
    }
  }

  // TODO(urneighbor1): テストをする
  public void updateLamp(boolean isEnabled) {
    boolean isScheduled = isScheduled();
    idleSignal.set(isEnabled && !isScheduled);
    runningSignal.set(isEnabled && isScheduled);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("startButton", startButton::getValue, null);
    builder.addDoubleProperty("emergencyButton", emergencyButton::getValue, null);
    builder.addDoubleProperty("resetButton", resetButton::getValue, null);
    builder.addDoubleProperty("bumperSensor", bumperSensor::getValue, null);
    builder.addBooleanProperty("idleLamp", idleSignal::get, idleSignal::set);
    builder.addBooleanProperty("runningLamp", runningSignal::get, runningSignal::set);
  }
}
