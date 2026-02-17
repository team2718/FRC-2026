package frc.robot.utils;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class SK6812RGBW extends AddressableLED {
  private AddressableLEDBuffer rgbwBuffer;
  private int[] rgbwValues;
  private int whiteValue = 0;

  public SK6812RGBW(int port) {
    super(port);

    setBitTiming(300, 900, 600, 600);
    setSyncTime(100);
    setColorOrder(ColorOrder.kRGB);
  }

  public void setWhiteValue(int whiteValue) {
    this.whiteValue = Math.max(0, Math.min(255, whiteValue));
  }

  /*
   * We are overriding setData to convert the RGB buffer into a new RGB buffer
   * that will trick the SK6812RGBW into working.
   */
  @Override
  public void setData(AddressableLEDBuffer rgbBuffer) {
    if (rgbBuffer == null) {
      throw new IllegalArgumentException("rgbBuffer cannot be null");
    }

    // Use the RGB values from the original buffer mixed with the white value
    // to emulate the order that the SK6812RGBW expects.
    if (rgbwValues == null || rgbwBuffer == null || rgbwValues.length < rgbBuffer.getLength() * 4) {
      setLength(rgbBuffer.getLength());
    }

    for (int i = 0; i < rgbBuffer.getLength(); i++) {
      rgbwValues[i * 4] = rgbBuffer.getGreen(i);
      rgbwValues[i * 4 + 1] = rgbBuffer.getRed(i);
      rgbwValues[i * 4 + 2] = rgbBuffer.getBlue(i);
      rgbwValues[i * 4 + 3] = whiteValue;
    }

    // The above array creates a repeating pattern of RGBW.
    // Now we make a AddressableLEDBuffer that takes those values three at a time,
    // which will appear like RGB WRG BWR GBW RGB...
    for (int i = 0; i < rgbwBuffer.getLength(); i++) {
      rgbwBuffer.setRGB(i, rgbwValues[i * 3], rgbwValues[i * 3 + 1], rgbwValues[i * 3 + 2]);
    }

    super.setData(rgbwBuffer);
  }

  @Override
  public void setLength(int length) {

    if (length < 0) {
      throw new IllegalArgumentException("Length cannot be negative");
    }

    if (length > 5460) {
      throw new IllegalArgumentException("Length cannot be greater than 5460");
    }

    super.setLength((int) Math.ceil(length * 4.0 / 3.0));

    rgbwValues = new int[length * 4];
    rgbwBuffer = new AddressableLEDBuffer((int) Math.ceil(length * 4.0 / 3.0));
  }

}