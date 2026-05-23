using igvc_csharp.src.Utils;
using OpenCvSharp;

namespace igvc_csharp.src.Subsystems.Feelers;

public class Feeler
{
    public SCR_Point Current;
    public SCR_Point Max;

    public Scalar Color { get; set; } = new(100, 100, 100);

    public Feeler()
    {
        Current = new SCR_Point(0, 0);
        Max = new SCR_Point(0, 0);
    }

    public Feeler(SCR_Point p)
    {
        Current = p;
        Max = p;
    }

    public Feeler(SCR_Point p, Scalar color) : this(p)
    {
        Color = color;
    }

    public override string ToString()
    {
        return $"({Current.X}, {Current.Y})";
    }

    public void Draw(Mat image)
    {
        image.Line(
            CenterCoordinates(new SCR_Point(0, 0), image.Width, image.Height).GetOpenCvPoint(),
            CenterCoordinates(Current, image.Width, image.Height).GetOpenCvPoint(),
            Lerp(Color), //FIXME who do we want to control color interpolation???
            3
        );
    }

    /**
     * Update the end of the feeler / its length from the image
     * It goes pixel by pixel along its length until it reaches a white pixel (obstacle)
     * or until it reaches its original length in which case it stops.
     * This was copied/pasted from feeler.cpp, for more information see
     * https://github.com/SoonerRobotics/autonav_software_2025/blob/main/autonav_ws/src/autonav_feelers/src/feeler.cpp
     * 
     * FIXME TODO we should pass this a pointer not the whole matrix so we can throw it into some threads.
     */
    public void Update(Mat image)
    {
        int channels = image.Channels();

        //FIXME we should like, automatically convert it maybe? Idk if I like throwing exceptions in what is basically a util class...
        if (channels > 1)
        {
            throw new ArgumentException("Image must be single-channel (i.e. grayscale)!");
        }

        bool success = image.GetArray(out byte[] raw_pixels); //FIXME move this outside Update() and have the feeler node itself call it?

        int x = 0;
        int y = 0;

        int last_x = 0;
        int last_y = 0;

        // a -1 signifies that the coordinate of the point lies along the negative axis
        int x_dir = Max.X < 0 ? -1 : 1;
        int y_dir = Max.Y < 0 ? -1 : 1;

        double slope = 0;
        bool slopeIsInfinity = (Max.X == 0);

        if (!slopeIsInfinity)
        {
            slope = Max.Y / Max.X; // rise over run
        }

        // loop until we hit an obstacle or max_length
        bool quit_checking = false;
        while (!quit_checking) //FIXME having infinite loops sketches me out ALSO FIXME pass in the cancellation token and check it would be good
        {
            // vertical line, just need to move along the y-axis
            if (slopeIsInfinity)
            {
                y += 1;
            }
            // horizontal line, move along the x axis
            else if (slope == 0)
            {
                x += 1;
            }
            // mostly horizontal line, move along the x-axis every step, and sometimes move along the y axis
            else if (Math.Abs(slope) <= 1)
            {
                // if the new y is higher than the previous one
                if (((Math.Abs(slope) * x) - last_y) > 0)
                {
                    y += 1; // then go up by 1 y
                }

                x += 1;
                last_y = y;
            }
            // last option is mostly vertical line, so move along y-axis every step, and if the slope calls for it, along the x-axis
            else
            {
                // and then if the new x is larger than the old one
                if (((Math.Abs(1 / slope) * y) - last_x) > 0)
                {
                    x += 1; // go up by one
                }

                y += 1;
                last_x = x;
            }

            // then center the coordinates
            var coords = CenterCoordinates(new SCR_Point(x * x_dir, y * y_dir), image.Cols, image.Rows);

            // check for out-of-bounds
            if (coords.X < 0 || coords.X > image.Cols || coords.Y < 0 || coords.Y > image.Rows)
            {
                // we *probably* exceeded our max, so just set it to max I guess
                Current = Max;
                quit_checking = true;
                continue;
            }

            // Console.WriteLine("(" + coords.X + ", " + coords.Y + ")");

            //FIXME make like a configurable obstacleThreshold like A*?
            if (raw_pixels[(coords.Y * image.Rows) + coords.X] > 0)
            {
                // that is our new length
                Current.X = x * x_dir;
                Current.Y = y * y_dir;

                quit_checking = true; // and quit so we don't keep looping 'cause we found an obstacle

                // Console.WriteLine("Got stopped at pixel: (" + coords.X + ", " + coords.Y + ") with value: " + raw_pixels[(coords.Y * image.Cols) + coords.X][channel]);
                continue;
            }
            else if (Math.Abs(x) > Math.Abs(Max.X) || Math.Abs(y) > Math.Abs(Max.Y))
            {
                // we've gone farther than our maximum, which means we didn't hit an obstacle, so reset
                Current = Max;

                // and quit checking
                quit_checking = true;
                continue;
            }
        }
    }

    public void Bias(double amount)
    {
        //FIXME we're gonna have to reintroduce the _original_unbiased_max again probably
        // Max += (Max * amount);
        // if (amount > 0)
        // {
        //     Max *= amount;
        // }
    }

    /// <summary>
    /// Copied and pasted from https://github.com/SoonerRobotics/autonav_software_2025/blob/main/autonav_ws/src/autonav_feelers/src/feeler.cpp
    /// flip y coordinate, because if the top-left corner of an image is the origin, 
    /// then the x axis will still work like normal (left is negative, etc) but the y axis will not
    /// </summary>
    /// <param name="p"></param>
    /// <param name="width"></param>
    /// <param name="height"></param>
    /// <returns></returns>
    public static SCR_Point CenterCoordinates(SCR_Point p, int width, int height)
    {
        return new SCR_Point(
            p.X + (width / 2),
            -p.Y + (int)(height * Configuration.FeelerSubsystem.YPercentage)
        );
    }

    public Scalar Lerp(Scalar otherColor)
    {
        //FIXME do we want to have feelers be transparent towards the center? maybe that will look better?

        //TODO actually write this part

        return otherColor;
    }

    //FIXME should these operate on CURRENT or MAX?
    public static Feeler operator +(Feeler a, Feeler b) => new(a.Current + b.Current, a.Color);
    public static Feeler operator -(Feeler a, Feeler b) => new(a.Current - b.Current, a.Color);

    // normalized dot product, should return like, the cosine of the angle between them
    public static double operator *(Feeler a, Feeler b)
    {
        // return (a.Max.X * b.Max.X) + (a.Max.Y * b.Max.Y);

        var aLength = a.Current.Dist(new SCR_Point());
        var aNormX = a.Current.X / aLength;
        var aNormY = a.Current.Y / aLength;

        var bLength = b.Current.Dist(new SCR_Point());
        var bNormX = b.Current.X / bLength;
        var bNormY = b.Current.Y / bLength;

        //FIXME shouldn't this only normalize one of the feelers? because it's supposed to be like vector projection or something right?
        return (aNormX * bNormX) + (aNormY * bNormY);
    }

    // scalar multiplication
    public static Feeler operator *(Feeler a, double scalar) => new(a.Current * scalar, a.Color);
}