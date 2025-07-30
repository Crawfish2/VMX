package frc.robot.util;

public class TransferFunction {
  /**
   * 転送関数：入力値を受け取り、対応する出力値を返す（線形補間を使用）
   *
   * @param findValue 変換する入力値
   * @param inputRange 入力値の範囲（降順ソート済み）
   * @param outputRange 出力値の範囲（降順ソート済み）
   * @return 変換された出力値
   * @throws IllegalArgumentException 入力範囲と出力範囲のサイズが異なる場合
   */
  public static double transferFunction(double findValue,
      final double[] inputRange,
      final double[] outputRange) {
    if (inputRange.length != outputRange.length) {
      throw new IllegalArgumentException("inputRange and outputRange must have the same size.");
    }

    int index = exploreLinear(findValue, inputRange);

    // 線形補間
    if (index + 1 < inputRange.length &&
        inputRange[index] < findValue &&
        findValue < inputRange[index + 1]) {

      double ratio = (findValue - inputRange[index]) /
          (inputRange[index + 1] - inputRange[index]);
      return outputRange[index] +
          (outputRange[index + 1] - outputRange[index]) * ratio;
    }

    return outputRange[index];
  }

  /**
   * 入力値が入力範囲のどの区間に属するかを線形探索で見つける
   *
   * @param findValue 探索する値
   * @param range 探索対象の範囲
   * @return 見つかった区間のインデックス
   */
  private static int exploreLinear(double findValue, final double[] range) {
    int size = range.length;

    // 範囲外チェック
    if (findValue < range[0])
      return 0;
    if (findValue > range[size - 1])
      return size - 1;

    // 線形探索（小さい配列サイズに最適化）
    for (int i = 0; i < size - 1; i++) {
      if (findValue <= range[i + 1]) {
        return i;
      }
    }

    return size - 1;
  }
}
