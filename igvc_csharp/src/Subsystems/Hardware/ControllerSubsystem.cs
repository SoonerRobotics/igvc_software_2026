using System.Runtime.InteropServices;
using igvc_csharp.CanSpec;
using igvc_csharp.Core;
using igvc_csharp.Core.Hardware;
using Microsoft.Extensions.Logging;

namespace igvc_csharp.Subsystems.Hardware;

[Subsystem("ControllerSubsystem", Disabled = false)]
public class ControllerSubsystem : SubsystemBase
{
    private XboxController? _controller;

    // Public Stuff
    public ControllerButtons Buttons { get; } = new();
    public ControllerAxes Axes { get; } = new();
    public ControllerDpad Dpad { get; } = new();

    public override Task Init(CancellationToken token)
    {
        _ = Task.Run(
            () => ReadLoop(token),
            token
        );
        
        return Task.CompletedTask;
    }

    public void DebugPrints()
    {
        Buttons.A.OnPressed += () => Logger.LogDebug("(Xbox Controller) A Button Pressed");
        Buttons.B.OnPressed += () => Logger.LogDebug("(Xbox Controller) B Button Pressed");
        Buttons.X.OnPressed += () => Logger.LogDebug("(Xbox Controller) X Button Pressed");
        Buttons.Y.OnPressed += () => Logger.LogDebug("(Xbox Controller) Y Button Pressed");
        Buttons.LeftStick.OnPressed += () => Logger.LogDebug("(Xbox Controller) Left Stick Pressed");
        Buttons.RightStick.OnPressed += () => Logger.LogDebug("(Xbox Controller) Right Stick Pressed");
        Buttons.LeftBumper.OnPressed += () => Logger.LogDebug("(Xbox Controller) Left Bumper Pressed");
        Buttons.RightBumper.OnPressed += () => Logger.LogDebug("(Xbox Controller) Right Bumper Pressed");
        Buttons.Menu.OnPressed += () => Logger.LogDebug("(Xbox Controller) Menu Button Pressed");
        Buttons.View.OnPressed += () => Logger.LogDebug("(Xbox Controller) View Button Pressed");
        Buttons.Xbox.OnPressed += () => Logger.LogDebug("(Xbox Controller) Xbox Button Pressed");
    }

    private XboxController? GetController()
    {
        if (_controller is { IsConnected: true })
        {
            return _controller;
        }

        var controllers = XboxController.GetControllers();
        if (controllers.Count == 0)
        {
            return null;
        }

        Logger.LogDebug("Controllers: {List}", controllers);
        _controller = new XboxController(controllers[^1]);
        return _controller;
    }

    private async Task ReadLoop(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            _controller ??= GetController();
            if (_controller == null)
            {
                SetState(SubsystemState.Ready);
                await Task.Delay(1000, token);
                continue;
            }

            Logger.LogWarning("Connecting to Xbox Controller -> {Path}", _controller.DevicePath);
            _controller.OnConnected += OnControllerConnected;
            _controller.OnDisconnected += OnControllerDisconnected;
            _controller.OnEvent += OnControllerEvent;

            SetState(SubsystemState.Operating);
            await _controller.ConnectAsync(token);
            SetState(SubsystemState.Ready);
            await Task.Delay(1000, token);
        }
    }

    private void OnControllerConnected(XboxController controller)
    {
        Logger.LogWarning("Xbox Controller Connected");
    }

    private void OnControllerDisconnected(XboxController controller)
    {
        Logger.LogWarning("Xbox Controller Disconnected");
    }

    private void OnControllerEvent(XboxControllerEvent e)
    {
        switch (e)
        {
            case XboxButtonEvent tn:
            {
                var action = tn.Button switch
                {
                    XboxButton.A => Buttons.A,
                    XboxButton.B => Buttons.B,
                    XboxButton.X => Buttons.X,
                    XboxButton.Y => Buttons.Y,
                    XboxButton.Menu => Buttons.Menu,
                    XboxButton.View => Buttons.View,
                    XboxButton.Xbox => Buttons.Xbox,
                    XboxButton.LeftStick => Buttons.LeftStick,
                    XboxButton.RightStick => Buttons.RightStick,
                    XboxButton.RightBumper => Buttons.RightBumper,
                    XboxButton.LeftBumper => Buttons.LeftBumper,
                    _ => null
                };

                action?.Update(tn.IsDown);
                break;
            }

            case XboxAxisEvent ae:
            {
                switch (ae.Axis)
                {
                    case XboxAxis.LeftTrigger or XboxAxis.RightTrigger:
                    {
                        var action = ae.Axis switch
                        {
                            XboxAxis.LeftTrigger => Axes.LeftTrigger,
                            XboxAxis.RightTrigger => Axes.RightTrigger,
                            _ => null
                        };
                        action?.Update(ae.Value);
                        break;
                    }
                    case XboxAxis.LeftX:
                        Axes.LeftStick.UpdateX(ae.Value);
                        break;
                    case XboxAxis.RightX:
                        Axes.RightStick.UpdateX(ae.Value);
                        break;
                    case XboxAxis.LeftY:
                        Axes.LeftStick.UpdateY(ae.Value);
                        break;
                    case XboxAxis.RightY:
                        Axes.RightStick.UpdateY(ae.Value);
                        break;
                }

                break;
            }

            case XboxDpadEvent dpe:
            {
                switch (dpe.X)
                {
                    case -1:
                        Dpad.DpadLeft.Update(true);
                        Dpad.DpadRight.Update(false);
                        break;
                    case 1:
                        Dpad.DpadRight.Update(true);
                        Dpad.DpadLeft.Update(false);
                        break;
                }

                switch (dpe.Y)
                {
                    case -1:
                        Dpad.DpadUp.Update(true);
                        Dpad.DpadDown.Update(false);
                        break;
                    case 1:
                        Dpad.DpadDown.Update(true);
                        Dpad.DpadUp.Update(false);
                        break;
                }

                break;
            }
        }
    }

    // Abstraction

    public sealed class InputAction
    {
        private bool _isDown;
        private bool _wasDown;

        public event Action? OnPressed;
        public event Action? OnReleased;
        public event Action? WhileHeld;

        public void Update(bool isDown)
        {
            _wasDown = _isDown;
            _isDown = isDown;

            if (_isDown && !_wasDown)
                OnPressed?.Invoke();

            if (!_isDown && _wasDown)
                OnReleased?.Invoke();

            if (_isDown)
                WhileHeld?.Invoke();
        }

        public bool IsDown => _isDown;
    }

    public sealed class VariableInputAction
    {
        private float _value;

        public event Action<float>? OnChanged;

        public void Update(float value)
        {
            _value = value;
            OnChanged?.Invoke(value);
        }
    }

    public sealed class MultiAxisInputAction
    {
        private float _x;
        private float _y;

        public event Action<float, float>? OnChanged;

        public void UpdateX(float x)
        {
            _x = x;
            OnChanged?.Invoke(_x, _y);
        }

        public void UpdateY(float y)
        {
            _y = y;
            OnChanged?.Invoke(_x, _y);
        }
    }

    public sealed class ControllerButtons
    {
        public InputAction A { get; } = new();
        public InputAction B { get; } = new();
        public InputAction X { get; } = new();
        public InputAction Y { get; } = new();
        public InputAction Menu { get; } = new();
        public InputAction View { get; } = new();
        public InputAction Xbox { get; } = new();
        public InputAction RightStick { get; } = new();
        public InputAction LeftStick { get; } = new();
        public InputAction RightBumper { get; } = new();
        public InputAction LeftBumper { get; } = new();
    }

    public sealed class ControllerAxes
    {
        public VariableInputAction LeftTrigger { get; } = new();
        public VariableInputAction RightTrigger { get; } = new();
        public MultiAxisInputAction LeftStick { get; } = new();
        public MultiAxisInputAction RightStick { get; } = new();
    }

    public sealed class ControllerDpad
    {
        public InputAction DpadLeft { get; } = new();
        public InputAction DpadRight { get; } = new();
        public InputAction DpadUp { get; } = new();
        public InputAction DpadDown { get; } = new();
    }
}