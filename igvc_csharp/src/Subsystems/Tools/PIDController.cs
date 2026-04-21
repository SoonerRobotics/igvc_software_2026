namespace igvc_csharp.Subsystems.Tools;

public class PIDController(double kP, double kI, double kD)
{
    private double KP { get; set; } = kP;
    private double KI { get; set; } = kI;
    private double KD { get; set; } = kD;

    private double Setpoint { get; set; } = 0.0;
    private double LastError { get; set; } = 0.0;
    private double LastTime { get; set; } = 0.0;

    public void SetSetpoint(double setpoint)
    {
        this->Setpoint = setpoint;
    }

    public double Calculate(double reading)
    {
        double timeElapsed = 0.0;
        if (LastTime == 0.0)
        {
            timeElapsed = 0.1; //FIXME
        }
        else
        {
            timeElapsed = LastTime - TimeUtils.Now();
        }
        LastTime = now();

        double error = Setpoint - reading;

        double errorDerivative = (error - LastError) / timeElapsed;

        LastError = error;

        return (KP * error) + (KI * 0.0 /*TODO FIXME*/) + (KD * errorDerivative);
    }

    public bool AtSetpoint()
    {
        return LastError < 10.0;
    }
};