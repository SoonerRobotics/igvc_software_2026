using System.Drawing;
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

    public void Draw(Mat image)
    {
        image.Line(
            CenterCoordinates(new SCR_Point(0, 0), image.Rows, image.Cols).GetOpenCvPoint(),
            CenterCoordinates(Current, image.Rows, image.Cols).GetOpenCvPoint(),
            Lerp(Color), //FIXME who do we want to control color interpolation???
            2 //FIXME line width
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
        bool success = image.GetArray<int>(out int[] raw_pixels); //FIXME move this outside Update() and have the feeler node itself call it?

        int x = 0;
        int y = 0;

        int last_x = 0;
        int last_y = 0;

        // a -1 signifies that the coordinate of the point lies along the negative axis
        int x_dir = Max.X < 0 ? -1 : 1;
        int y_dir = Max.Y < 0 ? -1 : 1;

        double slope = 0;
        bool slopeIsInfinity = (Max.Y == 0);

        if (slopeIsInfinity)
        {
            slope = Max.Y / Max.X; // rise over run
        }

        // loop until we hit an obstacle or max_length
        bool quit_checking = false;
        while (!quit_checking) //FIXME having infinite loops sketches me out
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
            var coords = CenterCoordinates(new SCR_Point(x * x_dir, y * y_dir), image.Rows, image.Cols);

            // check for out-of-bounds
            if (coords.X < 0 || coords.X > image.Rows || coords.Y < 0 || coords.Y > image.Cols)
            {
                // we *probably* exceeded our max, so just set it to max I guess
                Current = Max;
                quit_checking = true;
            }

            // for every one of the pixel's values (3 for the default BGR for OpenCV, although we should be sending only a B&W image so it should be 1)
            for (int channel = 0; channel < channels; channel++)
            {
                //reference https://stackoverflow.com/questions/7899108/opencv-get-pixel-channel-value-from-mat-image
                if (raw_pixels[(coords.X * image.Cols * channels) + (coords.Y * channels) + channel] > 0)
                {
                    // that is our new length
                    Current.X = x * x_dir;
                    Current.Y = y * y_dir;

                    quit_checking = true; // and quit so we don't keep looping 'cause we found an obstacle
                }
                else if (Math.Abs(x) > Math.Abs(Max.X)  ||  Math.Abs(y) > Math.Abs(Max.Y))
                {
                    // we've gone farther than our maximum, which means we didn't hit an obstacle, so reset
                    Current = Max;

                    // and quit checking
                    quit_checking = true;
                }
            }
        }
    }

    public void Bias(double amount)
    {
        Max *= amount;
    }

    //FIXME if we are only doing 1 forward camera and no others then we should change this to only center the x coordinate and have the y coordinates be elsewhere
    // or make it configurable or something is probably the better answer...
    public static SCR_Point CenterCoordinates(SCR_Point p, int width, int height)
    {
        return new SCR_Point(
            p.X + width / 2,
            p.Y + height / 2
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

    // dot product
    public static double operator *(Feeler a, Feeler b)
    {
        return (a.Max.X * b.Max.X) + (a.Max.Y * b.Max.Y);
    }

    // scalar multiplication
    public static Feeler operator *(Feeler a, double scalar) => new(a.Current * scalar, a.Color);
}