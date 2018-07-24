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


    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor = null;

        /// <summary>
        /// Bitmap that will hold color information
        /// </summary>
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

        private readonly KinectSensorChooser sensorChooser = new KinectSensorChooser();

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

        private void KinectSensorOnAllFramesReady(object sender, AllFramesReadyEventArgs allFramesReadyEventArgs)
        {
            using (var colorImageFrame = allFramesReadyEventArgs.OpenColorImageFrame())
            {
                if (colorImageFrame == null)
                {
                    return;
                }

                colorImageFrame.CopyPixelDataTo(this.colorPixels);

                this.colorBitmap.Lock();
                this.colorBitmap.WritePixels(new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight),
                    this.colorPixels,
                    this.colorBitmap.PixelWidth * sizeof(int), 0);
                this.colorBitmap.Unlock();
            }

            bool bodyReceived = false;
            Skeleton[] skeletons = null;
            List<Point> points = new List<Point>();

            using (SkeletonFrame skeletonFrame = allFramesReadyEventArgs.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    bodyReceived = true;
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            if (bodyReceived)
            {
                foreach(var skel in skeletons)
                {
                    if (skel.TrackingState == SkeletonTrackingState.Tracked)
                    {
                        Joint handright = skel.Joints[JointType.HandRight];
                        Joint wristright = skel.Joints[JointType.WristRight];

                        ColorImagePoint handColor =
                        this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint(handright.Position,
                            ColorImageFormat.RgbResolution640x480Fps30);
                        ColorImagePoint wristColor =
                        this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint(wristright.Position,
                            ColorImageFormat.RgbResolution640x480Fps30);

                        points.Add(new Point(handColor.X, handColor.Y));
                        points.Add(new Point(wristColor.X, wristColor.Y));
                    }
                }
            }

            using (var depthFrame = allFramesReadyEventArgs.OpenDepthImageFrame())
            {
                if (depthFrame == null)
                {
                    return;
                }

            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                dc.DrawImage(this.colorBitmap, new Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));

                for (var i = 0;i < points.Count; i+=2)
                {
                    dc.DrawLine(this.penY, points[i], points[i+1]);
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));
            }

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

                    // This is the bitmap we'll display on-screen
                    this.colorBitmap = new WriteableBitmap(this.sensor.ColorStream.FrameWidth, this.sensor.ColorStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

                    // Add an event handler to be called whenever there is new color frame data
                    this.sensor.AllFramesReady += KinectSensorOnAllFramesReady;
                }
                catch (InvalidOperationException)
                {
                    // This exception can be thrown when we are trying to
                    // enable streams on a device that has gone away.  This
                    // can occur, say, in app shutdown scenarios when the sensor
                    // goes away between the time it changed status and the
                    // time we get the sensor changed notification.
                    //
                    // Behavior here is to just eat the exception and assume
                    // another notification will come along if a sensor
                    // comes back.
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
        //private void WindowLoaded(object sender, RoutedEventArgs e)
        //{
        //    // Look through all sensors and start the first connected one.
        //    // This requires that a Kinect is connected at the time of app startup.
        //    // To make your app robust against plug/unplug, 
        //    // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
        //    foreach (var potentialSensor in KinectSensor.KinectSensors)
        //    {
        //        if (potentialSensor.Status == KinectStatus.Connected)
        //        {
        //            this.sensor = potentialSensor;
        //            break;
        //        }
        //    }

        //    if (null != this.sensor)
        //    {
        //        // Turn on the color stream to receive color frames
        //        this.sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);

        //        // Allocate space to put the pixels we'll receive
        //        this.colorPixels = new byte[this.sensor.ColorStream.FramePixelDataLength];

        //        // This is the bitmap we'll display on-screen
        //        this.colorBitmap = new WriteableBitmap(this.sensor.ColorStream.FrameWidth, this.sensor.ColorStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

        //        // Set the image we display to point to the bitmap where we'll put the image data
        //        this.Image.Source = this.colorBitmap;

        //        // Add an event handler to be called whenever there is new color frame data
        //        this.sensor.ColorFrameReady += this.SensorColorFrameReady;

        //        // Start the sensor!
        //        try
        //        {
        //            this.sensor.Start();
        //        }
        //        catch (IOException)
        //        {
        //            this.sensor = null;
        //        }
        //    }

        //    if (null == this.sensor)
        //    {
        //        this.statusBarText.Text = Properties.Resources.NoKinectReady;
        //    }
        //}

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