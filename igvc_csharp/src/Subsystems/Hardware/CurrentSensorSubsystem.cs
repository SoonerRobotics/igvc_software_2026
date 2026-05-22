using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Arc;
using igvc_csharp.Subsystems.Hardware.CanLayers;
using igvc_csharp.Utils.Messages;
using Messages.Arc;
using Microsoft.Extensions.Logging;
using SocketCANSharp;

namespace igvc_csharp.Subsystems.Hardware;

[Subsystem("CurrentSensorSubsystem")]
public class CurrentSensorSubsystem(
    CanbusSubsystem canbus,
    ChronosSubsystem? chronos
) : SubsystemBase
{
    private readonly SubsystemProperty<double> _pCurrentMa = new("current", 1200);
    private readonly SubsystemProperty<double> _pVoltageMv = new("voltage", 1200);

    public override Task Init(CancellationToken token)
    {
        Subscribe<CanFrameEvent>(OnCanMessage, token);
        
        return Task.CompletedTask;
    }

    private Task OnCanMessage(CanFrameEvent frameEvent, CancellationToken token)
    {
        var frame = frameEvent.Frame;
        switch (frame.CanId)
        {
            case (uint)CanId.SSD_CURRENT:
            {
                var currentMa = CurrentSensorLayer.ParseCurrent(frame);
                chronos?.WriteCurrentSense(currentMa);
                _pCurrentMa.Set(currentMa);
                // Logger.LogInformation("Current sensor reading: {Current} amps", currentMa);
                break;
            }
            case (uint)CanId.SSD_VOLTAGE:
            {
                var currentMv = CurrentSensorLayer.ParseVoltage(frame);
                chronos?.WriteVoltageSense(currentMv);
                _pVoltageMv.Set(currentMv);
                // Logger.LogInformation("Voltage sensor reading: {Voltage} volts", currentMv);
                break;
            }
        }
        
        return Task.CompletedTask;
    }   
    
    // Arc Commands
    
    [ArcCommand(ArcCommandId.CurrentSensorInitialize)]
    public void InitializeCurrentSensor()
    {
        canbus.CurrentSensor.Initialize();
        Logger.LogWarning("Initialized current sensor");
    }
    
    [ArcCommand(ArcCommandId.CurrentSensorSetBaudrate)]
    public void SetBaudrateCurrentSensor()
    {
        canbus.CurrentSensor.SetBaudrate();
        Logger.LogWarning("Updated current sensor baudrate");
    }
    
    [ArcCommand(ArcCommandId.CurrentSensorSaveEeprom)]
    public void SaveCurrentSensorEeprom()
    {
        canbus.CurrentSensor.SaveToEeprom();
        Logger.LogWarning("Saved current sensor settings to EEPROM");
    }
}