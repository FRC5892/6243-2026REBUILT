package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
  @AutoLog
  public static class LEDIOInputs {
    public boolean connected = true;
    public int ledCount = 0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(LEDIOInputs inputs) {}

  /** Set the LEDs to a solid RGB color. */
  default void setRGB(int r, int g, int b) {}

  /** Set the LEDs to a rainbow pattern. */
  default void setRainbow() {}

  /** Set the LEDs to a rainbow pattern with a breathing/fade brightness effect. */
  default void setRainbowFade() {
    setRainbow();
  }

  /** Set LED strip length. */
  default void setLength(int length) {}

  /** Turn off all LEDs. */
  default void clear() {}
}
