package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import java.util.Arrays;
import java.util.Map;

import frc.robot.Constants;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

public class ExampleSubsystem extends SubsystemBase {

  private TitanQuad[] motors;
  private TitanQuadEncoder[] motors_enc;
  private NetworkTableEntry[] MotorsEncoderValue;
  private NetworkTableEntry[] MotorsLimH_Value;
  private NetworkTableEntry[] MotorsLimL_Value;

  private NetworkTableEntry globalSpeed;
  private NetworkTableEntry globalAngle;

  public ShuffleboardTab tab = Shuffleboard.getTab("VMX");

  public double angle = 0.0;

  public ExampleSubsystem() {
    motors = new TitanQuad[4];
    motors_enc = new TitanQuadEncoder[4];
    MotorsEncoderValue = new NetworkTableEntry[4];
    MotorsLimH_Value = new NetworkTableEntry[4];
    MotorsLimL_Value = new NetworkTableEntry[4];

    globalSpeed = tab.add("Global Speed", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1.0, "max", 1.0))
        .getEntry();

    globalAngle = tab.add("Global Angle", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -360.0, "max", 360.0))
        .getEntry();

    for (int i = 0; i < 4; i++) {
      motors[i] = new TitanQuad(Constants.TITAN_ID, i);
      motors_enc[i] = new TitanQuadEncoder(motors[i], i, Constants.WHEEL_DIST_PER_TICK);

      MotorsEncoderValue[i] = tab.add("M" + i, 0).getEntry();
      MotorsLimH_Value[i] = tab.add("M" + i + " nLim.H", 0).getEntry();
      MotorsLimL_Value[i] = tab.add("M" + i + " nLim.L", 0).getEntry();

      motors_enc[i].reset();
    }
  }

  @Override
  public void periodic() {
    double speed = globalSpeed.getDouble(0.0);
    double angle = globalAngle.getDouble(0.0);

    this.angle = angle;

    drive(speed, angle);

    for (int i = 0; i < 4; i++) {
      MotorsEncoderValue[i].setDouble(motors_enc[i].getEncoderDistance());
      MotorsLimH_Value[i].setDouble(motors[i].getLimitSwitch(i, false));
      MotorsLimL_Value[i].setDouble(motors[i].getLimitSwitch(i, true));
    }
  }

  public void drive(double speed, double angle) {
    double[] motorsSpeed = new double[3];
    motorsSpeed[0] = Math.sin(Math.toRadians(angle + 180));
    motorsSpeed[1] = Math.sin(Math.toRadians(angle - 60));
    motorsSpeed[2] = Math.sin(Math.toRadians(angle + 60));

    double x = Arrays.stream(motorsSpeed).map(s -> Math.abs(s)).max().getAsDouble();

    motorsSpeed[0] = motorsSpeed[0] * speed / x;
    motorsSpeed[1] = motorsSpeed[1] * speed / x;
    motorsSpeed[2] = motorsSpeed[2] * speed / x;

    motors[0].set(motorsSpeed[0]);
    motors[1].set(motorsSpeed[1]);
    motors[2].set(motorsSpeed[2]);
  }
}
