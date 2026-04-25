
using igvc_csharp.Core;
using igvc_csharp.Events;
using igvc_csharp.Utils;
using OpenCvSharp;

// FIXME: add a "using namespace" or something here

public class Feeler
{
    private int _x = 0;
    private int _y = 0;
    private double _length = 0.0;

    private int _original_x = 0;
    private int _original_y = 0;
    private double _original_length = 0.0;

    private int _biased_x = 0;
    private int _biased_y = 0;
    private bool _is_biased = false;
    private double _bias_amount = 0.0;

    private Scalar _color;


    /**
     * Feeler constructor. Takes integer arguments for fast math and because the image uses integer coordinate for pixels.
     * Acts like a math vector of type <a, b> where (a, b) is the terminal point of the 2-d vector.
     * (0, 0) is assumed to be the center of the image
     * length is autocalulated and should never be manually set
     * @param x the x component of the feeler
     * @param y the y component of the feeler
     */
    public Feeler(int x, int y)
    {
        _x = x;
        _y = y;
        _length = Dist(x, y);

        // this is necessary so we can remember our original size and grow back up to it in the Math.Absence of an obstacle
        _original_x = x;
        _original_y = y;
        _original_length = _length;

        _color = new Scalar(100, 100, 100); //FIXME default color
    }

    /**
     * Feeler constructor with customizable color
     * @param x the x component of the feeler
     * @param y the y component of the feeler
     * @param color the color (as an OpenCv Scalar, in RGB) to use for the Feeler
     */
    public Feeler(int x, int y, Scalar color)
    {
        _x = x;
        _y = y;
        _length = Dist(x, y);

        // this is necessary so we can remember our original size and grow back up to it in the Math.Absence of an obstacle
        _original_x = x;
        _original_y = y;
        _original_length = _length;

        _color = color;
    }

    /**
     * @return x coordinate of the end of the feeler
     */
    public int GetX()
    {
        return _x;
    }

    /**
     * @return y coordinate of the end of the feeler
     */
    public int GetY()
    {
        return _y;
    }

    /**
     * @return the calculated length of the feeler as it currently stands
     */
    public double GetLength()
    {
        return Math.Sqrt((_x * _x) + (_y * _y));
    }

    /**
     * Set the color of the feeler. This is important if it gets drawn on an image on the UI for debugging
     * @param the color of the feeler, in BGR because OpenCV
     */
    public void SetColor(Scalar c)
    {
        _color = c;
    }

    /**
     * Get the polar coordinates of the end of the feeler from its x and y coordinates in radians.
     * @return the polar coordinates of the feeler in radians
     */
    public List<double> ToPolar()
    {
        List<double> polar = [];

        double length = Dist(_x, _y);

        double angle;
        if (_x != 0)
        {
            //TODO FIXME should this be an atan2?
            angle = Math.Tan(_y / _x);
        }
        else
        {
            angle = _y > 0 ? Math.PI / 2 : 3 * Math.PI / 2;
        }

        polar.Add(length);
        polar.Add(angle);

        return polar;
    }

    /**
     * Get the coordinates of a pixel in top-left origin (opencv) from assuming origin is the center of the image (feelers)
     * @param x the x coordinate to translate
     * @param y the y coordiante to translate
     * @return the coordinates of the pixel in opencv-land
     */
    public static List<double> CenterCoordinates(int x, int y, int width, int height)
    {
        List<double> ret = [];

        ret.Add(x + width / 2);
        ret.Add(-y + height / 2); // flip y coordinate, because if the top-left corner of an image is the origin, then the x axis will still work like normal (left is negative, etc) but the y axis will not

        return ret;
    }

    public static double Dist(int x, int y)
    {
        return Math.Sqrt((x * x) + (y * y));
    }

    /**
     * @return x coordinate of the end of the feeler when it was created
     */
    public int GetOriginalX()
    {
        return _original_x;
    }

    /**
     * @return y coordinate of the end of the feeler when it was created
     */
    public int GetOriginalY()
    {
        return _original_y;
    }

    /**
     * @return length of the feeler when it was created
     */
    public double GetOriginalLength()
    {
        return _original_length;
    }

    /**
     * @return the bias amount (total) of the feeler
     */
    public double GetBiasAmount()
    {
        return _bias_amount;
    }

    /**
     * @return color of the feeler (for drawing purposes)
     */
    public Scalar GetColor()
    {
        return _color;
    }

    /**
     * TODO figure out which data we actually want to have here
     * @return a string representation of the feeler
     */
    public override String ToString()
    {
        return "(" + _x + ", " + _y + ") | length " + _length + " | original: " + _original_length;
    }

    /**
     * Set the x and y of the end point of the feeler, relative to the origin
     * the origin being the center of the image
     */
    public void SetXY(int x_, int y_)
    {
        _x = x_;
        _y = y_;

        _length = Dist(x_, y_);
    }

    /**
     * Set the original x and y of the feeler. This is mostly used for all the operator magic methods (*, -, +)
     */
    public void SetOriginalXY(int x, int y)
    {
        _original_x = x;
        _original_y = y;

        _original_length = Dist(x, y);
    }

    /**
     * Set the length of the feeler (don't call this directly for the vision feelers, they should set their own length via update())
     * This may or may not work but I think this is how vectors work
     * @param length the length of the feeler
     */
    public void SetLength(double newLength)
    {
        double scaleFactor = newLength / _length;

        _x *= (int)scaleFactor;
        _y *= (int)scaleFactor;
        _length = Dist(_x, _y);
    }

    /**
     * Biases the feeler by an amount of pixels.
     * This essentially allows the feeler to grow past its original maximum length,
     * for the purposes of navigating towards a waypoint/biasing 'forward'/etc
     * @param amount the amount to bias by, in pixels
     */
    public void Bias(double amount)
    {
        //FIXME THIS SHOULD BE ADDITIVE!! AND THERE SHOULD BE A .ResetBias() METHOD!!!

        // unbias by setting it to 0
        if (amount <= 0.0)
        {
            _is_biased = false;
            _bias_amount = 0.0;
            return;
        }

        _is_biased = true;

        _bias_amount = amount;
        double scaleFactor = (_length + amount) / _length;

        _biased_x = (int)(_original_x * scaleFactor);
        _biased_y = (int)(_original_y * scaleFactor);
    }

    /**
     * Update the end of the feeler / its length from the image
     * It goes pixel by pixel along its length until it reaches a white pixel (obstacle)
     * or until it reaches its original length in which case it stops.
     * This was copied/pasted from feeler.py, see feeler.py for details
     * And now it's been copied/pasted from feeler.cpp, check that one first
     * FIXME TODO we should pass this a pointer not the whole matrix so we can throw it into some threads.
     * @param the thresholded image to perform feeler on
     */
    public void Update(Mat mask)
    {
        int channels = mask.Channels();
        var pixelPtr = mask.Data;

        int x_ = 0;
        int y_ = 0;

        int prev_x = 0;
        int prev_y = 0;

        double new_y = 0;
        double new_x = 0;

        int x_dir = _original_x < 0 ? -1 : 1;
        int y_dir = _original_y < 0 ? -1 : 1;

        double slope = 0;
        bool slopeIsInfinity = false;
        if (_original_y != 0)
        {
            slope = _original_y / _original_x;
        }
        else
        {
            slopeIsInfinity = true;
        }

        int pixelsChecked = 0; //FIXME remove when done debugging

        // loop until we hit an obstacle or max_length
        while (true)
        {
            // vertical line, just need to move along the y-axis
            if (slopeIsInfinity)
            {
                y_ += 1;
            }
            else if (slope == 0)
            {
                x_ += 1;
            }
            else if (Math.Abs(slope) <= 1)
            {
                // if slope is shallow, make x the independent variable
                // get the y as a function of x
                new_y = Math.Abs(slope) * x_;

                // if the new y is higher than the previous one
                if ((new_y - prev_y) > 0)
                {
                    y_ += 1; // then go up by 1 y
                }

                x_ += 1;
                prev_y = y_;
            }
            else
            { // slope is steep, do y as independent variable
              // get x as a function of y
                new_x = Math.Abs(1 / slope) * y_;

                // and then if the new x is larger than the old one
                if ((new_x - prev_x) > 0)
                {
                    x_ += 1; // go up by one
                }

                y_ += 1;
                prev_x = x_;
            }

            var coords = CenterCoordinates(x_ * x_dir, y_ * y_dir, mask.Cols, mask.Rows);

            // for every one of the pixel's values (out of blue, green, and red as per openCV standard)
            pixelsChecked++;
            for (int i = 0; i < 3; i++)
            {
                //reference https://stackoverflow.com/questions/7899108/opencv-get-pixel-channel-value-from-mat-image
                if (pixelPtr[coords[1] * mask.Cols * channels + coords[0] * channels + i] > 0)
                {
                    // that is our new length
                    SetXY(x_ * x_dir, y_ * y_dir);
                    return; // and quit so we don't keep looping 'cause we found an obstacle
                }
                // if we're past our original farthest point
                else if (Math.Abs(x_) > Math.Abs(_original_x) || Math.Abs(y_) > Math.Abs(_original_y))
                {
                  // check if we have a farther point (aka if we're biased)
                    if (_is_biased)
                    {
                        if (Math.Abs(x_) > Math.Abs(_biased_x) || Math.Abs(y_) > Math.Abs(_biased_y))
                        {
                            SetXY(_biased_x, _biased_y); // then we found no obstacle, and should stop looping
                            return;
                        }
                    }
                    else
                    {
                        SetXY(_original_x, _original_y); // then we found no obstacle, and should stop looping
                        return;
                    }
                }
            }
        }
    }

    /**
     * Draw the feeler using its color on the provided image.
     * @param image an image that the feeler can be drawn on
     */
    public void Draw(Mat image)
    {
        Point startPt = new();
        Point endPt = new();

        var startCoords = CenterCoordinates(0, 0, image.Cols, image.Rows);
        startPt.X = (int)startCoords[0];
        startPt.Y = (int)startCoords[1];

        var endCoords = CenterCoordinates(_x, _y, image.Cols, image.Rows);
        endPt.X = (int)Math.Clamp(endCoords[0], 0, image.Cols);
        endPt.Y = (int)Math.Clamp(endCoords[1], 0, image.Rows);

        Line(image, startPt, endPt, _color, 5); // thickness of 5
    }


    /**
     * Interpolates (linearly) between two colors. For some reason openCV doesn't have a function like this?
     * At least not one I could find. Both colors should have the same number of channels.
     * https://stackoverflow.com/questions/4353525/floating-point-linear-interpolation
     * @param src the base color
     * @param dest the color to interpolate towards
     * @param percentAmount a number between 0 and 1 inclusive to represent how much each color contributes to the final color
     * @return an openCV color somewhere between the two given colors, inclusive
     */
    public static Scalar Lerp(Scalar src, Scalar dest, double percentAmount)
    {
        double ch1 = (src[0] * (1.0 - percentAmount)) + (dest[0] * percentAmount);
        double ch2 = (src[1] * (1.0 - percentAmount)) + (dest[1] * percentAmount);
        double ch3 = (src[2] * (1.0 - percentAmount)) + (dest[2] * percentAmount);
        // ignore channel 4 'cause we shouldn't ever use it for Feelers (though potentially this we could use it for other things,
        //  as this is not a class method)

        return new Scalar(ch1, ch2, ch3);
    }


    /**
     * Add a feeler to another feeler
     * does basic vector addition of the type <a,b>+<c,d> = <a+c, b+d>
     * @param the feeler to add to this feeler
     * @return a new feeler with values copied from the current feeler plus the other feeler
     */
    public static Feeler operator +(Feeler self, Feeler other) => new(self.GetX() + other.GetX(), self.GetY() + other.GetY(), self.GetColor());

    /**
     * Subtract a feeler from another feeler
     * does basic vector subtraction of the type <a,b>-<c,d> = <a-c, b-d>
     * @param the feeler to subtract from this feeler
     * @return a new feeler with values copied from the current feeler minus the other feeler
     */
    public static Feeler operator -(Feeler self, Feeler other) => new(self.GetX() - other.GetX(), self.GetY() - other.GetY(), self.GetColor());

    /**
     * Multiply a feeler by a scalar value.
     * This works exactly like in math. 
     * Could do with a double version too.
     * @param scalarNum an integer to multiply by
     * @return a new feeler with values copied from the old one
     */
    public static Feeler operator *(Feeler self, double scalar) => new((int)(self.GetX() * scalar), (int)(self.GetY() * scalar), self.GetColor());

    /**
     * Dot product a feeler with another feeler.
     * This works exactly like in math. v1 = <x1, y1>; v2 = <x2, y2>
     * result = (x1*x2) + (y1*y2) 
     * @param other the other Feeler
     * @return a number representing the dot product of the two Feelers.
     */
    // public static int operator *(Feeler self, Feeler other) => (self.GetX() * other.GetX()) + (self.GetY() * other.GetY());

    /**
     * Dot product a feeler with another feeler, but normalize both of them first.
     * So we're essentially just getting the angle of each.
     * This works exactly like in math. v1 = |<x1, y1>|; v2 = |<x2, y2>|
     * result = v1 dot v2
     * @param other the other Feeler
     * @return a number representing the dot product of the two Feelers after both have been normalized.
     */
    public static double operator *(Feeler self, Feeler other)
    {
        if (other.GetX() == 0 && other.GetY() == 0)
        {
            return 0.0;
        }

        double x_norm = self.GetX() / self.GetLength();
        double y_norm = self.GetY() / self.GetLength();

        double other_x_norm = other.GetX() / other.GetLength();
        double other_y_norm = other.GetY() / other.GetLength();

        return (x_norm * other_x_norm) + (y_norm * other_y_norm);
    }
}