//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Palmpose360
{
    using System;
    using System.Collections.Generic;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using Microsoft.Kinect.Toolkit;
    using System.Runtime.InteropServices;

    [StructLayout(LayoutKind.Sequential)]
    public struct PointXYZ
    {
        public float x;
        public float y;
        public float z;
        private float placeholder;
        public PointXYZ(float x, float y, float z)
        {
            this.x = x; this.y = y; this.z = z; this.placeholder = 0.0f;
        }
    }

    public class ExternalAPI
    {
#if DEBUG
        [DllImport("../../Debug/PointCloud.dll", EntryPoint = "#1", CallingConvention = CallingConvention.Cdecl)]
#else
        [DllImport("../../Release/PointCloud.dll", EntryPoint = "#1", CallingConvention = CallingConvention.Cdecl)]
#endif
        public static extern int infer_palmpose(ref PointXYZ v1, ref PointXYZ v2, ref PointXYZ v3, [In, Out] PointXYZ[] pointArray, ref int size, ref PointXYZ hand, ref PointXYZ wrist);

    }

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private const DepthImageFormat DepthFormat = DepthImageFormat.Resolution320x240Fps30;
        private const ColorImageFormat ColorFormat = ColorImageFormat.RgbResolution640x480Fps30;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor = null;

        /// <summary>
        /// Bitmap that will hold color information
        /// </summary>
        /// 
        private WriteableBitmap colorBitmap;
        private DrawingGroup drawingGroup;
        private DrawingImage imageSource;
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        private readonly Brush anyBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));
        private readonly Pen penX = new Pen(Brushes.Red, 3);
        private readonly Pen penY = new Pen(Brushes.Green, 3);
        private readonly Pen penZ = new Pen(Brushes.Blue, 3);

        /// <summary>
        /// Intermediate storage for the color data received from the camera
        /// </summary>
        private byte[] colorPixels;
        private DepthImagePixel[] depthPixels;
        private int[] playerPixelData;
        private ColorImagePoint[] colorCoordinates;
        /// Inverse scaling factor between color and depth
        private int colorToDepthDivisor;


        private readonly KinectSensorChooser sensorChooser = new KinectSensorChooser();
        private int depthWidth;
        private int depthHeight;
        private int colorWidth;
        private int colorHeight;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            this.drawingGroup = new DrawingGroup();
            this.imageSource = new DrawingImage(this.drawingGroup);

            InitializeComponent();
            sensorChooser.KinectChanged += SensorChooserOnKinectChanged;
            sensorChooser.Start();

        }

        private void SensorChooserOnKinectChanged(object sender, KinectChangedEventArgs kinectChangedEventArgs)
        {
            KinectSensor oldSensor = kinectChangedEventArgs.OldSensor;
            KinectSensor newSensor = kinectChangedEventArgs.NewSensor;

            if (oldSensor != null)
            {
                oldSensor.AllFramesReady -= KinectSensorOnAllFramesReady;
                oldSensor.ColorStream.Disable();
                oldSensor.DepthStream.Disable();
                oldSensor.DepthStream.Range = DepthRange.Default;
                oldSensor.SkeletonStream.Disable();
                oldSensor.SkeletonStream.EnableTrackingInNearRange = false;
                oldSensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
            }

            if (newSensor != null)
            {
                this.sensor = newSensor;

                try
                {
                    newSensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                    newSensor.DepthStream.Enable(DepthImageFormat.Resolution320x240Fps30);
                    newSensor.SkeletonStream.Enable();
                    this.depthWidth = this.sensor.DepthStream.FrameWidth;
                    this.depthHeight = this.sensor.DepthStream.FrameHeight;
                    this.colorWidth = this.sensor.ColorStream.FrameWidth;
                    this.colorHeight = this.sensor.ColorStream.FrameHeight;
                    this.colorToDepthDivisor = colorWidth / this.depthWidth;

                    try
                    {
                        // This will throw on non Kinect For Windows devices.
                        newSensor.DepthStream.Range = DepthRange.Near;
                        newSensor.SkeletonStream.EnableTrackingInNearRange = true;
                        //newSensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                    }
                    catch (InvalidOperationException)
                    {
                        newSensor.DepthStream.Range = DepthRange.Default;
                        newSensor.SkeletonStream.EnableTrackingInNearRange = false;
                    }

                    // Allocate space to put the pixels we'll receive
                    this.colorPixels = new byte[this.sensor.ColorStream.FramePixelDataLength];
                    this.depthPixels = new DepthImagePixel[this.sensor.DepthStream.FramePixelDataLength];
                    this.colorCoordinates = new ColorImagePoint[this.sensor.DepthStream.FramePixelDataLength];
                    this.playerPixelData = new int[this.sensor.DepthStream.FramePixelDataLength];


                    // This is the bitmap we'll display on-screen
                    this.colorBitmap = new WriteableBitmap(this.colorWidth, this.colorHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

                    // Add an event handler to be called whenever there is new color frame data
                    this.sensor.AllFramesReady += KinectSensorOnAllFramesReady;
                }
                catch (InvalidOperationException)
                {
                }
            }
            else
            {
                this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }
        }


        private void KinectSensorOnAllFramesReady(object sender, AllFramesReadyEventArgs allFramesReadyEventArgs)
        {
            if (null == this.sensor)
            {
                return;
            }

            bool depthReceived = false;
            bool colorReceived = false;
            bool bodyReceived = false;

            Skeleton[] skeletons = null;
            List<Point> points = new List<Point>();
            Point handPointColor = new Point() { };
            Point wristPointColor = new Point() { };
            PointXYZ handXYZ = new PointXYZ();
            PointXYZ wristXYZ = new PointXYZ();


            using (SkeletonFrame skeletonFrame = allFramesReadyEventArgs.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    bodyReceived = true;
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            int leftBound = 0;
            int topBound = 0;
            int rightBound = 0;
            int bottomBound = 0;

            if (bodyReceived)
            {
                foreach (var skel in skeletons)
                {
                    if (skel.TrackingState == SkeletonTrackingState.Tracked)
                    {
                        Joint handLeft = skel.Joints[JointType.HandLeft];
                        Joint wristLeft = skel.Joints[JointType.WristLeft];
                        Joint shoulder0 = skel.Joints[JointType.ShoulderLeft];
                        Joint shoulder1 = skel.Joints[JointType.ShoulderRight];

                        var shoulder0Color = this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint(shoulder0.Position, ColorImageFormat.RgbResolution640x480Fps30);
                        var shoulder1Color = this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint(shoulder1.Position, ColorImageFormat.RgbResolution640x480Fps30);
                        float bodyScaleColor = 0.4f * (float)Math.Sqrt(
                            (shoulder1Color.X - shoulder0Color.X) * (shoulder1Color.X - shoulder0Color.X)
                            + (shoulder1Color.Y - shoulder0Color.Y) * (shoulder1Color.Y - shoulder0Color.Y));

                        ColorImagePoint handColor =
                        this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint(handLeft.Position,
                            ColorImageFormat.RgbResolution640x480Fps30);
                        ColorImagePoint wristColor =
                        this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint(wristLeft.Position,
                            ColorImageFormat.RgbResolution640x480Fps30);

                        leftBound = (int)(handColor.X - bodyScaleColor);
                        topBound = (int)(handColor.Y - bodyScaleColor);
                        rightBound = (int)(handColor.X + bodyScaleColor);
                        bottomBound = (int)(handColor.Y + bodyScaleColor);

                        handPointColor = new Point(handColor.X, handColor.Y);
                        wristPointColor = new Point(wristColor.X, wristColor.Y);
                        handXYZ = new PointXYZ(handLeft.Position.X, handLeft.Position.Y, handLeft.Position.Z);
                        wristXYZ = new PointXYZ(wristLeft.Position.X, wristLeft.Position.Y, wristLeft.Position.Z);

                        // quit after handling one player
                        // assuming only one player
                        break;
                    }
                }
            }

            using (DepthImageFrame depthFrame = allFramesReadyEventArgs.OpenDepthImageFrame())
            {
                if (null != depthFrame)
                {
                    // Copy the pixel data from the image to a temporary array
                    depthFrame.CopyDepthImagePixelDataTo(this.depthPixels);
                    depthReceived = true;
                }
            }


            List<int> colorIndices = new List<int>();
            List<SkeletonPoint> candidatePoints = new List<SkeletonPoint>();
            Dictionary<SkeletonPoint, int> pointColorDict = new Dictionary<SkeletonPoint, int>();

            if (true == depthReceived && true == bodyReceived)
            {
                this.sensor.CoordinateMapper.MapDepthFrameToColorFrame(
                    DepthFormat,
                    this.depthPixels,
                    ColorFormat,
                    this.colorCoordinates);


                // loop over each row and column of the depth
                for (int y = 0; y < this.depthHeight; ++y)
                {
                    for (int x = 0; x < this.depthWidth; ++x)
                    {
                        // calculate index into depth array
                        int depthIndex = x + (y * this.depthWidth);

                        DepthImagePixel depthPixel = this.depthPixels[depthIndex];
                        int player = depthPixel.PlayerIndex;
                        // assuming one player
                        if (player > 0 && depthPixel.IsKnownDepth)
                        {
                            // retrieve the depth to color mapping for the current depth pixel
                            ColorImagePoint colorImagePoint = this.colorCoordinates[depthIndex];

                            if (colorImagePoint.X >= leftBound && colorImagePoint.X < rightBound
                                && colorImagePoint.Y >= topBound && colorImagePoint.Y < bottomBound)
                            {
                                int colorIndex = colorImagePoint.X + colorImagePoint.Y * colorWidth;
                                colorIndices.Add(colorIndex);

                                DepthImagePoint dp = new DepthImagePoint() { X = x, Y = y, Depth = depthPixel.Depth };
                                SkeletonPoint p = this.sensor.CoordinateMapper.MapDepthPointToSkeletonPoint(DepthFormat, dp);
                                candidatePoints.Add(p);

                                pointColorDict.Add(p, colorIndex);
                            }
                        }
                    }
                }
            }

            using (var colorImageFrame = allFramesReadyEventArgs.OpenColorImageFrame())
            {
                if (colorImageFrame == null)
                {
                    return;
                }

                colorImageFrame.CopyPixelDataTo(this.colorPixels);
                colorReceived = true;
            }

            if (true == depthReceived && true == bodyReceived && true == colorReceived)
            {
                int nPoint = candidatePoints.Count;

                if (nPoint > 20)
                {
                    PointXYZ[] pointCloud = new PointXYZ[nPoint];
                    for (int i = 0; i < nPoint; i++)
                    {
                        pointCloud[i] = new PointXYZ(candidatePoints[i].X, candidatePoints[i].Y, candidatePoints[i].Z);
                    }

                    int pointCloudLength = pointCloud.Length;

                    PointXYZ v1 = new PointXYZ();
                    PointXYZ v2 = new PointXYZ();
                    PointXYZ v3 = new PointXYZ();

                    ExternalAPI.infer_palmpose(ref v1, ref v2, ref v3, pointCloud, ref pointCloudLength,
                        ref handXYZ, ref wristXYZ);


                    SkeletonPoint[] skelPoints = new SkeletonPoint[pointCloudLength];
                    ColorImagePoint[] newColorPoints = new ColorImagePoint[pointCloudLength];
                    for (int i = 0; i < pointCloudLength; i++)
                    {
                        skelPoints[i].X = pointCloud[i].x;
                        skelPoints[i].Y = pointCloud[i].y;
                        skelPoints[i].Z = pointCloud[i].z;
                    }

                    for(int i = 0; i < pointCloudLength; i++)
                    {
                        ColorImagePoint colorP = this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skelPoints[i], ColorFormat);
                        newColorPoints[i] = colorP;
                    }

                    foreach (var cp in newColorPoints)
                    {
                        {
                            int index = cp.X + cp.Y * this.colorWidth;
                            this.colorPixels[index * 4 + 0] = 0;
                            this.colorPixels[index * 4 + 1] = 255;
                            this.colorPixels[index * 4 + 2] = 0;
                            this.colorPixels[index * 4 + 3] = 255;

                        }
                    }

                    //int n_colorIndices = colorIndices.Count;
                    //for (int i = 0; i < n_colorIndices; i++)
                    //{
                    //    int index = colorIndices[i];
                    //    this.colorPixels[index * 4 + 0] = 0;
                    //    this.colorPixels[index * 4 + 1] = 255;
                    //    this.colorPixels[index * 4 + 2] = 0;
                    //    this.colorPixels[index * 4 + 3] = 255;
                    //}
                }


                this.colorBitmap.WritePixels(new Int32Rect(0, 0, this.colorWidth, this.colorHeight),
                    this.colorPixels,
                    this.colorBitmap.PixelWidth * sizeof(int), 0);

                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    dc.DrawImage(this.colorBitmap, new Rect(0, 0, this.colorWidth, this.colorHeight));

                    // if draw hands
                    if (bodyReceived)
                    {
                        dc.DrawLine(this.penX, handPointColor, wristPointColor);
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.colorWidth, this.colorHeight));
                }

            }
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            Image.Source = this.imageSource;
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            this.sensorChooser.Stop();

            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }



        /// <summary>
        /// Handles the user clicking on the screenshot button
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void ButtonScreenshotClick(object sender, RoutedEventArgs e)
        {
            if (null == this.sensor)
            {
                this.statusBarText.Text = Properties.Resources.ConnectDeviceFirst;
                return;
            }

            // create a png bitmap encoder which knows how to save a .png file
            BitmapEncoder encoder = new PngBitmapEncoder();

            // create frame from the writable bitmap and add to encoder
            encoder.Frames.Add(BitmapFrame.Create(this.colorBitmap));

            string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

            string path = Path.Combine(myPhotos, "KinectSnapshot-" + time + ".png");

            // write the new file to disk
            try
            {
                using (FileStream fs = new FileStream(path, FileMode.Create))
                {
                    encoder.Save(fs);
                }

                this.statusBarText.Text = string.Format(CultureInfo.InvariantCulture, "{0} {1}", Properties.Resources.ScreenshotWriteSuccess, path);
            }
            catch (IOException)
            {
                this.statusBarText.Text = string.Format(CultureInfo.InvariantCulture, "{0} {1}", Properties.Resources.ScreenshotWriteFailed, path);
            }
        }
    }
}