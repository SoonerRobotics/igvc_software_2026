using Google.FlatBuffers;
using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Subsystems.Arc;
using igvc_csharp.Subsystems.Hardware.CanLayers;
using igvc_csharp.Utils.Messages;
using Messages;
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

    private bool _isLowWarningActive = false;

    public override Task Init(CancellationToken token)
    {
        Subscribe<CanFrameEvent>(OnCanMessage, token);

        return Task.CompletedTask;
    }

    public override Task OnRobotStateChanged(RobotState old, RobotState updated)
    {
        if (updated.Mode == RobotModeEnum.Disabled && old.Mode != RobotModeEnum.Disabled && _isLowWarningActive)
        {
            // wait a second for other safety lights to update, then set low power mode
            Task.Delay(1000).ContinueWith(_ => canbus.SafetyLights.SetLowPower());
        }

        return base.OnRobotStateChanged(old, updated);
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

                    // if it is below 11.8 volts, enable the warning
                    if (currentMv < 11.9 && !_isLowWarningActive && (
                        BaseRobot.Instance?.State.Mode == RobotModeEnum.Disabled
                    ))
                    {
                        Logger.LogWarning("Voltage is low: {Voltage} volts", currentMv);
                        _isLowWarningActive = true;

                        // if current mode is disabled, set low power mode
                        if (BaseRobot.Instance?.State.Mode == RobotModeEnum.Disabled)
                        {
                            // play low battery noise
                            var builder = new FlatBufferBuilder(128);
                            var fileOffset = builder.CreateString("amongus-sound.mp3");
                            var msg = AudibleFeedback.CreateAudibleFeedback(builder, fileOffset, false);
                            builder.Finish(msg.Value);
                            EventBus.Instance.Publish(new MessageWrapperEvent(
                                MessageWrapper.From(MessageType.AudibleFeedback, builder.SizedByteArray()))
                            );

                            // wait a second for other safety lights to update, then set low power mode
                            Task.Delay(1000).ContinueWith(_ => canbus.SafetyLights.SetLowPower());
                        }
                    }
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