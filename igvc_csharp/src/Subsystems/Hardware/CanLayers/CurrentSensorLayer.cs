using SocketCANSharp;

namespace igvc_csharp.Subsystems.Hardware.CanLayers;

public class CurrentSensorLayer(CanbusSubsystem canbus)
{
    private static readonly byte[] SetmodeData = [0x03, 0x12, 0xA3, 0x08];
    private static readonly byte[] SetIntervalData = [0x03, 0x16, 0x00, 0xFA];
    private static readonly byte[] SaveEepromData = [0x03, 0x10, 0x00, 0x0F];
    private static readonly byte[] SetA2DData = [0x03, 0x17, 0x03, 0x5B];
    private static readonly byte[] SetBaudRateData = [0x03, 0x14, 0x00, 0x09];

    public void Initialize()
    {
        canbus.SendCanFrame(new CanFrame(
            (uint)CanId.SSD_COMMAND,
            SetA2DData
        ));        
        
        canbus.SendCanFrame(new CanFrame(
            (uint)CanId.SSD_COMMAND,
            SetmodeData
        ));
        
        canbus.SendCanFrame(new CanFrame(
            (uint)CanId.SSD_COMMAND,
            SetIntervalData
        ));
    }

    public void SetBaudrate()
    {
        canbus.SendCanFrame(new CanFrame(
            (uint)CanId.SSD_COMMAND,
            SetBaudRateData
        ));
    }
    
    public void SaveToEeprom()
    {
        canbus.SendCanFrame(new CanFrame(
            (uint)CanId.SSD_COMMAND,
            SaveEepromData
        ));
    }

    public static double ParseCurrent(CanFrame frame)
    {
        if (frame.CanId != (uint)CanId.SSD_CURRENT)
        {
            throw new ArgumentException($"Invalid current CAN frame ID: {frame.CanId}");
        }

        return Math.Abs(BitConverter.ToInt32(frame.Data, 0) / 1000.0);
    }
    
    public static double ParseVoltage(CanFrame frame)
    {
        if (frame.CanId != (uint)CanId.SSD_VOLTAGE)
        {
            throw new ArgumentException($"Invalid voltage CAN frame ID: {frame.CanId}");
        }

        return Math.Abs(BitConverter.ToInt32(frame.Data, 0) / 1000.0);
    }
}