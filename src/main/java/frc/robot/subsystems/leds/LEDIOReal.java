package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Hardware implementation using WPILib AddressableLED. */
public class LEDIOReal implements LEDIO {
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
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, r, g, b);
    }
    led.setData(buffer);
  }

  @Override
  public void setRainbow() {
    // For every pixel
    for (var i = 0; i < buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
      // Set the value
      buffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    rainbowFirstPixelHue += 3;
    // Check bounds
    rainbowFirstPixelHue %= 180;

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
}
