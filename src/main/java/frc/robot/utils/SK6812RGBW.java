// Modified version of AddressableLED class to support SK6812RGBW.
// The original class only supported RGB LEDs, so this adds support for the white channel.
//
// The setData method has been modified to convert the GRB values from the buffer to the RGBW format expected by the hardware.
//
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.hal.AddressableLEDJNI;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.PWMJNI;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * A class for driving addressable LEDs, such as WS2812B, WS2815, and NeoPixels.
 *
 * <p>By default, the timing supports WS2812B and WS2815 LEDs, but is configurable using {@link
 * #setBitTiming(int, int, int, int)}
 *
 * <p>Some LEDs use a different color order than the default GRB. The color order is configurable
 * using {@link #setColorOrder(ColorOrder)}.
 *
 * <p>Only 1 LED driver is currently supported by the roboRIO. However, multiple LED strips can be
 * connected in series and controlled from the single driver.
 */
public class SK6812RGBW implements AutoCloseable {
  private final int m_pwmHandle;
  private final int m_handle;

  private int whiteValue = 0;

  /**
   * Constructs a new driver for a specific port.
   *
   * @param port the output port to use (Must be a PWM header, not on MXP)
   */
  public SK6812RGBW(int port) {
    m_pwmHandle = PWMJNI.initializePWMPort(HAL.getPort((byte) port));
    m_handle = AddressableLEDJNI.initialize(m_pwmHandle);
    HAL.report(tResourceType.kResourceType_AddressableLEDs, port + 1);

    setBitTiming(300, 900, 600, 600);
    setSyncTime(80);

    AddressableLEDJNI.setColorOrder(m_handle, AddressableLEDJNI.COLOR_ORDER_RGB);
  }

  @Override
  public void close() {
    if (m_handle != 0) {
      AddressableLEDJNI.free(m_handle);
    }
    if (m_pwmHandle != 0) {
      PWMJNI.freePWMPort(m_pwmHandle);
    }
  }

  /**
   * Sets the length of the LED strip.
   *
   * <p>Calling this is an expensive call, so it's best to call it once, then just update data.
   *
   * <p>The max length is 5460 LEDs.
   *
   * @param length the strip length
   */
  public void setLength(int length) {
    AddressableLEDJNI.setLength(m_handle, length);
  }

  /**
   * Sets the LED output data.
   *
   * <p>If the output is enabled, this will start writing the next data cycle. It is safe to call,
   * even while output is enabled.
   *
   * @param buffer the buffer to write
   */
  public void setData(AddressableLEDBuffer buffer) {

    byte[] bufferGRBW = new byte[buffer.getLength() * 4];
    for (int i = 0; i < buffer.getLength(); i++) {

        int r = buffer.getRed(i);
        int g = buffer.getGreen(i);
        int b = buffer.getBlue(i);

        bufferGRBW[i * 4] = (byte) g;
        bufferGRBW[i * 4 + 1] = (byte) r;
        bufferGRBW[i * 4 + 2] = (byte) b;
        bufferGRBW[i * 4 + 3] = (byte) whiteValue;
    }

    AddressableLEDJNI.setData(m_handle, bufferGRBW);
  }

  /**
   * Sets the bit timing.
   *
   * <p>By default, the driver is set up to drive WS2812B and WS2815, so nothing needs to be set for
   * those.
   *
   * @param highTime0NanoSeconds high time for 0 bit (default 400ns)
   * @param lowTime0NanoSeconds low time for 0 bit (default 900ns)
   * @param highTime1NanoSeconds high time for 1 bit (default 900ns)
   * @param lowTime1NanoSeconds low time for 1 bit (default 600ns)
   */
  private void setBitTiming(
      int highTime0NanoSeconds,
      int lowTime0NanoSeconds,
      int highTime1NanoSeconds,
      int lowTime1NanoSeconds) {
    AddressableLEDJNI.setBitTiming(
        m_handle,
        highTime0NanoSeconds,
        lowTime0NanoSeconds,
        highTime1NanoSeconds,
        lowTime1NanoSeconds);
  }

  /**
   * Sets the sync time.
   *
   * <p>The sync time is the time to hold output so LEDs enable. Default set for WS2812B and WS2815.
   *
   * @param syncTimeMicroSeconds the sync time (default 280us)
   */
  private void setSyncTime(int syncTimeMicroSeconds) {
    AddressableLEDJNI.setSyncTime(m_handle, syncTimeMicroSeconds);
  }

  /**
   * Starts the output.
   *
   * <p>The output writes continuously.
   */
  public void start() {
    AddressableLEDJNI.start(m_handle);
  }

  /** Stops the output. */
  public void stop() {
    AddressableLEDJNI.stop(m_handle);
  }

  /** Sets the white channel value for all LEDs. This will be applied to all LEDs in the next call to setData. */
  public void setWhite(int whiteValue) {
    this.whiteValue = whiteValue;
  }
}
