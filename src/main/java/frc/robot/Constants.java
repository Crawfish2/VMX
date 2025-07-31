
package frc.robot;

public final class Constants {

  public static class TitanConstants {
    public static final int TITAN_ID = 42;

    public static final int MOTOR0 = 0;
    public static final int MOTOR1 = 1;
    public static final int MOTOR2 = 2;
    public static final int MOTOR3 = 3;

    /** Encoder pulse per revolution */
    public static final double pulsePerRevolution = 1464;

    public static class DriveConstants {
      public static final int WHEEL_LEFT = MOTOR0;
      public static final int WHEEL_RIGHT = MOTOR2;
      public static final int WHEEL_BACK = MOTOR1;

      public static final int WHEEL_LEFT_ANGLE = 60;
      public static final int WHEEL_RIGHT_ANGLE = -60;
      public static final int WHEEL_BACK_ANGLE = 180;

      /** オムニホイールの取り付け位置のロボットの回転中心からの距離 (mm) */
      public static final double wheelDistanceFromCenter = 245.0;

      /** Omni wheel radius(mm) */
      public static final double wheelRadius = 51;
      /** Gear Ratio between encoder and wheel */
      public static final double gearRatio = 1 / 1;
      /** Pulse per revolution of wheel */
      public static final double wheelPulseRatio = pulsePerRevolution * gearRatio;
      /** Distance per tick */
      public static final double WHEEL_DIST_PER_TICK =
          (Math.PI * 2 * wheelRadius) / wheelPulseRatio;
    }

    public static class ElevatorConstants {
      public static final int ELEVATOR = MOTOR1;

      public static final int safeLimitSwitchPin = 2;

      /** gear radius(mm) */
      public static final double gearRadius = 6; // 3?
      /** Gear Ratio between encoder and wheel */
      public static final double gearRatio = 24.0 / 64;
      /** Pulse per revolution of wheel */
      public static final double wheelPulseRatio = pulsePerRevolution * gearRatio;
      /** Distance per tick */
      public static final double WHEEL_DIST_PER_TICK =
          (Math.PI * 2 * gearRadius) / wheelPulseRatio;
    }
  }

  public static class Ultrasonic {
    // https://docs.wsr.studica.com/en/latest/docs/Sensors/ultrasonic.html
    public static final int frontLeftPingPin = 10;
    public static final int frontRightPingPin = 8;

    public static final int middleLeftPingPin = 6;
    public static final int middleRightPingPin = 4;

    public static final int rearLeftPingPin = 2;
    public static final int rearRightPingPin = 0;

    public static final int frontLeftEchoPin = frontLeftPingPin + 1;
    public static final int frontRightEchoPin = frontRightPingPin + 1;
    public static final int middleLeftEchoPin = middleLeftPingPin + 1;
    public static final int middleRightEchoPin = middleRightPingPin + 1;
    public static final int rearLeftEchoPin = rearLeftPingPin + 1;
    public static final int rearRightEchoPin = rearRightPingPin + 1;
  }

  public static class ArmConstants {
    public static final int SERVO_CHANNEL = 0;
  }

  public static class StatusHandlerConstants {
    public static final int startButtonChannel = 0;
    public static final int resetButtonChannel = 1;
    public static final int emergencyButtonChannel = 2;
    public static final int bumperSensorChannel = 3;

    public static final int idleSignalChannel = 13;
    public static final int runningSignalChannel = 14;

  }


  /** 実行環境がVMX-Pi上かどうか */
  public static final boolean isReal =
      // VMX-PiではHALUtil.getHALRuntimeType()==1となり、
      // RobotBase.isReal()は正しい結果を返さない

      // OSのCPUアーキテクチャの情報を読んで、arm(VMX-Pi)かどうかを判定する
      // armのWindowsなどでは正しく動かないが、暫定的な対応
      System.getProperty("os.arch").contains("arm");

  public static final boolean isSimulation = !isReal;

}
