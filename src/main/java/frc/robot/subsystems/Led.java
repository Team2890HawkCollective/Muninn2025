package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;

public class Led {
    public static AddressableLED signalLighs =  new AddressableLED(Constants.LED.SIGNAL_LIGHTS_PORT);
    public static AddressableLEDBuffer signalLightsBuffer = new AddressableLEDBuffer(Constants.LED.SIGNAL_LIGHTS_LENGTH);

    public static int getBufferLength(AddressableLEDBuffer buffer){
        return buffer.getLength();
    }

    public  void setColor(AddressableLED led, AddressableLEDBuffer buffer, int r, int g, int b){
        for (var i = 0; i < getBufferLength(buffer); i++)
          buffer.setRGB(i, r, g, b);
        led.setData(buffer);
    }
}
