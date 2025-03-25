package frc.robot.util;

import java.util.Arrays;
import java.util.stream.IntStream;

import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

import frc.robot.Constants;

public class OmniDrive {
    private TitanQuad[] motors;
    private TitanQuadEncoder[] motors_enc;
    public static final int MOTOR_NUM = 3;

    public OmniDrive() {
        motors = IntStream.range(0, MOTOR_NUM).mapToObj(i -> new TitanQuad(Constants.TITAN_ID, i))
                .toArray(TitanQuad[]::new);
        motors_enc = IntStream.range(0, MOTOR_NUM)
                .mapToObj(i -> new TitanQuadEncoder(motors[i], i, Constants.WHEEL_DIST_PER_TICK))
                .toArray(TitanQuadEncoder[]::new);
        Arrays.stream(motors_enc).forEach(motor_enc -> motor_enc.reset());
    }

    /**
     * Will give the distance traveled based on encoder data
     *
     * @param motor Motor number
     * @return Distance traveled
     */
    public double getEncoderDistance(int motor) {
        return motors_enc[motor].getEncoderDistance();
    }

    /**
     * 指定された角度方向の移動距離を計算します
     *
     * @param angle 移動方向の角度（度数法）
     * @return 指定方向の移動距離
     */
    public double getDistance(double angle) {
        // 各モーターのエンコーダー距離を取得
        double d0 = getEncoderDistance(0);
        double d1 = getEncoderDistance(1);
        double d2 = getEncoderDistance(2);

        // 各ホイールの寄与を計算（move()メソッドの逆の計算）
        double contribution0 = d0 * Math.sin(Math.toRadians(angle + 180));
        double contribution1 = d1 * Math.sin(Math.toRadians(angle + 60));
        double contribution2 = d2 * Math.sin(Math.toRadians(angle - 60));

        // 3つの寄与の平均を取ることで、指定方向の距離を得る
        return (contribution0 + contribution1 + contribution2) / 3.0;
    }

    /**
     * Gets limit switch status for specified motor
     *
     * @param motor     Motor number
     * @param direction Direction (true: low side, false: high side)
     * @return Limit switch value
     */
    public double getLimitSwitch(int motor, boolean direction) {
        return motors[motor].getLimitSwitch(motor, direction);
    }

    private void setMotorsSpeed(double[] speeds) {
        for (int i = 0; i < MOTOR_NUM; i++) {
            motors[i].set(speeds[i]);
        }
    }

    /**
     * Moves the robot with specified speed and angle
     *
     * @param speed The speed to move (range -1.0 to 1.0)
     * @param angle The angle in degrees (0 degrees is forward)
     */
    public void move(double speed, double angle) {
        double[] motorsSpeed = new double[MOTOR_NUM];
        motorsSpeed[0] = Math.sin(Math.toRadians(angle + 180));
        motorsSpeed[1] = Math.sin(Math.toRadians(angle + 60));
        motorsSpeed[2] = Math.sin(Math.toRadians(angle - 60));

        double x = Arrays.stream(motorsSpeed).map(s -> Math.abs(s)).max().getAsDouble();

        // Normalize
        motorsSpeed[0] = motorsSpeed[0] * speed / x;
        motorsSpeed[1] = motorsSpeed[1] * speed / x;
        motorsSpeed[2] = motorsSpeed[2] * speed / x;

        setMotorsSpeed(motorsSpeed);
    }

    /**
     * Rotates the robot with specified speed
     *
     * @param speed The speed to rotate (range -1.0 to 1.0)
     */
    public void rotate(double speed) {
        double[] motorsSpeed = new double[MOTOR_NUM];
        Arrays.fill(motorsSpeed, speed);

        setMotorsSpeed(motorsSpeed);
    }

    /**
     * Stops the robot's wheels
     */
    public void stop() {
        double[] motorsSpeed = new double[MOTOR_NUM];
        Arrays.fill(motorsSpeed, 0.0);
        setMotorsSpeed(motorsSpeed);
    }
}
