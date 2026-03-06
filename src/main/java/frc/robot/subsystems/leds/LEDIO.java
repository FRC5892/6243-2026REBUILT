package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
  // TODO: Confirm final strip length/port values are wired from constants at robot startup.

  @AutoLog
  public static class LEDIOInputs {
    public boolean connected = true;
    public int ledCount = 0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(LEDIOInputs inputs) {}

  /** Set the LEDs to a solid RGB color. */
  // TODO: Clamp RGB values (0-255) in implementation to guard against bad caller values.
  default void setRGB(int r, int g, int b) {}

  /** Set the LEDs to a rainbow pattern. */
  // TODO: Add brightness/speed controls for rainbow mode once final driver feedback is in.
  default void setRainbow() {}

  /** Set LED strip length. */
  default void setLength(int length) {}

  /** Turn off all LEDs. */
  default void clear() {}
}
