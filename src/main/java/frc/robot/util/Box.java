package frc.robot.util;

/**
 * 値のラッパー
 */
public class Box<T> {
  public T value;

  public Box(T value) {
    this.value = value;
  }

  public void set(T value) {
    this.value = value;
  }

  public T get() {
    return value;
  }


  @Override
  public boolean equals(Object obj) {
    if (obj == this)
      return true;
    if (!(obj instanceof Box))
      return false;

    Box<?> other = (Box<?>) obj;

    if (this.value == null) {
      return other.value == null;
    }

    return this.value.equals(other.value);
  }
}
