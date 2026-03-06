package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

/** Hardware implementation using WPILib AddressableLED. */
public class LEDIOReal implements LEDIO {
  private static final int HUE_SPAN = 180;
  private static final int SATURATION_FULL = 255;
  private static final int RAINBOW_MIN_VALUE = 32;
  private static final int RAINBOW_MAX_VALUE = 255;
  private static final double RAINBOW_FADE_PERIOD_SEC = 1.6;

  private final AddressableLED led;
  private AddressableLEDBuffer buffer;
  private int rainbowFirstPixelHue = 0;

  /**
   * Creates a new hardware LED controller.
   *
   * @param pwmPort The PWM port for the LED strip
   * @param length Number of LEDs in the strip
   */
  public LEDIOReal(int pwmPort, int length) {
    led = new AddressableLED(pwmPort);
    buffer = new AddressableLEDBuffer(length);
    led.setLength(length);
    led.setData(buffer);
    led.start();
  }

  @Override
  public void updateInputs(LEDIOInputs inputs) {
    inputs.connected = true;
    inputs.ledCount = buffer.getLength();
  }

  @Override
  public void setRGB(int r, int g, int b) {
    final int clampedR = clampColor(r);
    final int clampedG = clampColor(g);
    final int clampedB = clampColor(b);
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, clampedR, clampedG, clampedB);
    }
    led.setData(buffer);
  }

  @Override
  public void setRainbow() {
    final int value = (RAINBOW_MIN_VALUE + RAINBOW_MAX_VALUE) / 2;

    // Keep the non-fade rainbow available for any callers that want fixed brightness.
    for (var i = 0; i < buffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * HUE_SPAN / buffer.getLength())) % HUE_SPAN;
      buffer.setHSV(i, hue, SATURATION_FULL, value);
    }

    rainbowFirstPixelHue = (rainbowFirstPixelHue + 3) % HUE_SPAN;
    led.setData(buffer);
  }

  @Override
  public void setRainbowFade() {
    final double seconds = Timer.getFPGATimestamp();
    final double fadeWave =
        0.5 * (1.0 + Math.sin((2.0 * Math.PI * seconds) / RAINBOW_FADE_PERIOD_SEC));
    final int value =
        (int) Math.round(RAINBOW_MIN_VALUE + fadeWave * (RAINBOW_MAX_VALUE - RAINBOW_MIN_VALUE));

    for (var i = 0; i < buffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * HUE_SPAN / buffer.getLength())) % HUE_SPAN;
      buffer.setHSV(i, hue, SATURATION_FULL, value);
    }

    rainbowFirstPixelHue = (rainbowFirstPixelHue + 3) % HUE_SPAN;
    led.setData(buffer);
  }

  @Override
  public void setLength(int length) {
    buffer = new AddressableLEDBuffer(length);
    led.setLength(length);
  }

  @Override
  public void clear() {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, 0, 0, 0);
    }
    led.setData(buffer);
  }

  private static int clampColor(int value) {
    return Math.max(0, Math.min(255, value));
  }
}
