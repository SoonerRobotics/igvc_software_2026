using igvc_csharp.Core;
using Microsoft.Extensions.Logging;
using Silk.NET.SDL;

namespace igvc_csharp.Subsystems.Hardware;

[Subsystem("ControllerSubsystem", Disabled = false)]
public class ControllerSubsystem : SubsystemBase
{
    private Sdl _sdl = null!;
    private unsafe GameController* _controller;
    private int _instanceId = -1;

    // Public Stuff
    public ControllerButtons Buttons { get; } = new();
    public ControllerAxes Axes { get; } = new();
    public ControllerDpad Dpad { get; } = new();

    private const double StickScale = 1.0 / 32767.0;
    private const double TriggerScale = 1.0 / 32767.0;

    public override Task Init(CancellationToken token)
    {
        _sdl = Sdl.GetApi();

        if (_sdl.Init(Sdl.InitGamecontroller) < 0)
        {
            var err = _sdl.GetErrorS();
            Logger.LogError("SDL_Init failed: {Error}", err);
            throw new InvalidOperationException($"SDL_Init failed: {err}");
        }

        _ = Task.Run(() => ReadLoop(token), token);

        // DebugPrints();
        return Task.CompletedTask;
    }

    public override Task Periodic(CancellationToken token)
    {
        if (Buttons.A.IsDown) Buttons.A.Update(true);
        if (Buttons.B.IsDown) Buttons.B.Update(true);
        if (Buttons.X.IsDown) Buttons.X.Update(true);
        if (Buttons.Y.IsDown) Buttons.Y.Update(true);
        if (Buttons.LeftStick.IsDown) Buttons.LeftStick.Update(true);
        if (Buttons.RightStick.IsDown) Buttons.RightStick.Update(true);
        if (Buttons.LeftBumper.IsDown) Buttons.LeftBumper.Update(true);
        if (Buttons.RightBumper.IsDown) Buttons.RightBumper.Update(true);
        if (Buttons.Menu.IsDown) Buttons.Menu.Update(true);
        if (Buttons.View.IsDown) Buttons.View.Update(true);
        if (Buttons.Xbox.IsDown) Buttons.Xbox.Update(true);

        if (Dpad.DpadLeft.IsDown) Dpad.DpadLeft.Update(true);
        if (Dpad.DpadRight.IsDown) Dpad.DpadRight.Update(true);
        if (Dpad.DpadUp.IsDown) Dpad.DpadUp.Update(true);
        if (Dpad.DpadDown.IsDown) Dpad.DpadDown.Update(true);

        Axes.LeftStick.Broadcast();
        Axes.RightStick.Broadcast();
        Axes.LeftTrigger.Broadcast();
        Axes.RightTrigger.Broadcast();

        return Task.CompletedTask;
    }

    private void DebugPrints()
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

        Dpad.DpadLeft.OnPressed += () => Logger.LogDebug("(Xbox Controller) Dpad Left Button Pressed");
        Dpad.DpadRight.OnPressed += () => Logger.LogDebug("(Xbox Controller) Dpad Right Button Pressed");
        Dpad.DpadDown.OnPressed += () => Logger.LogDebug("(Xbox Controller) Dpad Down Button Pressed");
        Dpad.DpadUp.OnPressed += () => Logger.LogDebug("(Xbox Controller) Dpad Up Button Pressed");

        Axes.LeftStick.OnChanged += (dirX, dirY) => Logger.LogDebug("(Xbox Controller) Left Stick: {x}.{y}", dirX, dirY);
        Axes.RightStick.OnChanged += (dirX, dirY) => Logger.LogDebug("(Xbox Controller) Right Stick: {x}.{y}", dirX, dirY);
        Axes.LeftTrigger.OnChanged += (dir) => Logger.LogDebug("(Xbox Controller) Left Trigger: {x}", dir);
        Axes.RightTrigger.OnChanged += (dir) => Logger.LogDebug("(Xbox Controller) Right Trigger: {x}", dir);
    }

    private unsafe bool TryOpenController()
    {
        if (_controller != null) return true;

        var count = _sdl.NumJoysticks();
        for (var i = 0; i < count; i++)
        {
            if (_sdl.IsGameController(i) == SdlBool.True)
            {
                _controller = _sdl.GameControllerOpen(i);
                if (_controller == null)
                {
                    Logger.LogWarning("Failed to open controller {Index}: {Err}", i, _sdl.GetErrorS());
                    continue;
                }

                var joystick = _sdl.GameControllerGetJoystick(_controller);
                _instanceId = _sdl.JoystickInstanceID(joystick);

                var namePtr = _sdl.GameControllerNameS(_controller);
                Logger.LogWarning("Connected to controller -> {Name} (instance {Id})", namePtr, _instanceId);
                return true;
            }
        }

        return false;
    }

    private unsafe void CloseController()
    {
        if (_controller == null) return;
        _sdl.GameControllerClose(_controller);
        _controller = null;
        _instanceId = -1;
        Logger.LogWarning("Xbox Controller Disconnected");

        ReleaseAllInputs();
    }

    private void ReleaseAllInputs()
    {
        Buttons.A.Update(false); Buttons.B.Update(false);
        Buttons.X.Update(false); Buttons.Y.Update(false);
        Buttons.LeftStick.Update(false); Buttons.RightStick.Update(false);
        Buttons.LeftBumper.Update(false); Buttons.RightBumper.Update(false);
        Buttons.Menu.Update(false); Buttons.View.Update(false); Buttons.Xbox.Update(false);
        Dpad.DpadLeft.Update(false); Dpad.DpadRight.Update(false);
        Dpad.DpadUp.Update(false); Dpad.DpadDown.Update(false);
        Axes.LeftStick.UpdateX(0); Axes.LeftStick.UpdateY(0);
        Axes.RightStick.UpdateX(0); Axes.RightStick.UpdateY(0);
        Axes.LeftTrigger.Update(0); Axes.RightTrigger.Update(0);
    }

    private async Task ReadLoop(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            if (!TryOpenController())
            {
                SetOperatingState(SubsystemState.Idle);
                PumpEvents();
                await Task.Delay(500, token);
                continue;
            }

            SetOperatingState(SubsystemState.Operating);
            PumpEvents();

            await Task.Delay(4, token);
        }

        CloseController();
        _sdl.Quit();
    }

    private unsafe void PumpEvents()
    {
        Event e;
        while (_sdl.PollEvent(&e) == 1)
        {
            switch ((EventType)e.Type)
            {
                case EventType.Controllerdeviceadded:
                    break;

                case EventType.Controllerdeviceremoved:
                    if (e.Cdevice.Which == _instanceId)
                        CloseController();
                    break;

                case EventType.Controllerbuttondown:
                case EventType.Controllerbuttonup:
                    HandleButton(e.Cbutton);
                    break;

                case EventType.Controlleraxismotion:
                    HandleAxis(e.Caxis);
                    break;
            }
        }
    }

    private void HandleButton(ControllerButtonEvent be)
    {
        var isDown = be.State == 1;
        var button = (GameControllerButton)be.Button;

        InputAction? action = button switch
        {
            GameControllerButton.A => Buttons.A,
            GameControllerButton.B => Buttons.B,
            GameControllerButton.X => Buttons.X,
            GameControllerButton.Y => Buttons.Y,
            GameControllerButton.Back => Buttons.View,
            GameControllerButton.Start => Buttons.Menu,
            GameControllerButton.Guide => Buttons.Xbox,
            GameControllerButton.Leftstick => Buttons.LeftStick,
            GameControllerButton.Rightstick => Buttons.RightStick,
            GameControllerButton.Leftshoulder => Buttons.LeftBumper,
            GameControllerButton.Rightshoulder => Buttons.RightBumper,
            GameControllerButton.DpadUp => Dpad.DpadUp,
            GameControllerButton.DpadDown => Dpad.DpadDown,
            GameControllerButton.DpadLeft => Dpad.DpadLeft,
            GameControllerButton.DpadRight => Dpad.DpadRight,
            _ => null
        };

        action?.Update(isDown);
    }

    private void HandleAxis(ControllerAxisEvent ae)
    {
        var axis = (GameControllerAxis)ae.Axis;
        var raw = ae.Value;

        switch (axis)
        {
            case GameControllerAxis.Leftx:
                Axes.LeftStick.UpdateX(raw * StickScale);
                break;
            case GameControllerAxis.Lefty:
                Axes.LeftStick.UpdateY(-raw * StickScale);
                break;
            case GameControllerAxis.Rightx:
                Axes.RightStick.UpdateX(raw * StickScale);
                break;
            case GameControllerAxis.Righty:
                Axes.RightStick.UpdateY(-raw * StickScale);
                break;
            case GameControllerAxis.Triggerleft:
                Axes.LeftTrigger.Update(raw * TriggerScale);
                break;
            case GameControllerAxis.Triggerright:
                Axes.RightTrigger.Update(raw * TriggerScale);
                break;
        }
    }

    // Abstraction (unchanged)

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

            if (_isDown && !_wasDown) OnPressed?.Invoke();
            if (!_isDown && _wasDown) OnReleased?.Invoke();
            if (_isDown) WhileHeld?.Invoke();
        }

        public bool IsDown => _isDown;
    }

    public sealed class VariableInputAction
    {
        private double _value;
        public event Action<double>? OnChanged;

        public void Update(double value)
        {
            _value = value;
            OnChanged?.Invoke(value);
        }

        public void Broadcast() => OnChanged?.Invoke(_value);
    }

    public sealed class MultiAxisInputAction
    {
        private double _x;
        private double _y;
        public event Action<double, double>? OnChanged;

        public void UpdateX(double x) { _x = x; OnChanged?.Invoke(_x, _y); }
        public void UpdateY(double y) { _y = y; OnChanged?.Invoke(_x, _y); }
        public void Broadcast() => OnChanged?.Invoke(_x, _y);
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