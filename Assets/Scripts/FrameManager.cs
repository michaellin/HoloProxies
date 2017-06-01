using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Windows.Kinect;
using System.IO;

using HoloProxies.Utils;

namespace HoloProxies.Objects
{
    /// <summary>
	/// The FrameManager class interfaces with the Kinect
    /// File from: ISRFrame.h + LowLevelEngine
    /// </summary>
    public class FrameManager
    {
		public UnityEngine.Vector4 boundingbox;

        public int DownsampleSize = 1;

        public int Width { get; private set; }
        public int Height { get; private set; }

        public int ColorWidth { get; private set; }
        public int ColorHeight { get; private set; }

        public int DepthWidth { get; private set; }
        public int DepthHeight { get; private set; }

        public CameraSpacePoint[] Camera3DPoints { get; private set; }
        public float[] PfVec { get; private set; }
        public Texture2D PfImage { get; private set; }

        public Texture2D ColorTexture { get; private set; }
        public ColorSpacePoint[] ColorPoints { get; private set; }
        public byte[] ColorData { get; private set; }

		public Color[] Mask { get; private set; }

        public ushort[] DepthData { get; private set; }

        private KinectSensor _Sensor;
        private MultiSourceFrameReader _Reader;
        private CoordinateMapper _Mapper;

        private ColorHistogram histogram;

        // Constructor
        public FrameManager( ColorHistogram hist )
        {
            _Sensor = KinectSensor.GetDefault();

            if (_Sensor != null)
            {
                // Save histogram
                histogram = hist;

                _Mapper = _Sensor.CoordinateMapper;

                _Reader = _Sensor.OpenMultiSourceFrameReader( FrameSourceTypes.Color | FrameSourceTypes.Depth );

                var colorFrameDesc = _Sensor.ColorFrameSource.CreateFrameDescription( ColorImageFormat.Rgba );
                ColorWidth = colorFrameDesc.Width;
                ColorHeight = colorFrameDesc.Height;
                ColorData = new byte[colorFrameDesc.BytesPerPixel * colorFrameDesc.LengthInPixels];
                ColorTexture = new Texture2D( ColorWidth, ColorHeight, TextureFormat.RGBA32, false );

                var depthFrameDesc = _Sensor.DepthFrameSource.FrameDescription;
                DepthWidth = depthFrameDesc.Width;
                DepthHeight = depthFrameDesc.Height;
                DepthData = new ushort[depthFrameDesc.LengthInPixels];

                // TODO
                // Downsample all data frames if necessary

                // Set FrameWidth and FrameHeight to the downsampled size
                Width = DepthWidth;
                Height = DepthHeight;

                PfImage = new Texture2D( Width, Height, TextureFormat.RGBA32, false );

                // Set buffers to align depth data to RGB and to align camera points
                ColorPoints = new ColorSpacePoint[DepthWidth * DepthHeight];
                Camera3DPoints = new CameraSpacePoint[DepthWidth * DepthHeight];
                PfVec = new float[Width * Height];
				Mask = new Color[Width * Height];

                if (!_Sensor.IsOpen)
                {
                    _Sensor.Open();
                }
            }
        }

        public void UpdateFrame()
        {

            if (_Reader != null)
            {
                var frame = _Reader.AcquireLatestFrame();
                if (frame != null)
                {
                    var colorFrame = frame.ColorFrameReference.AcquireFrame();
                    if (colorFrame != null)
                    {
                        var depthFrame = frame.DepthFrameReference.AcquireFrame();
                        if (depthFrame != null)
                        {
                            colorFrame.CopyConvertedFrameDataToArray( ColorData, ColorImageFormat.Rgba );
                            ColorTexture.LoadRawTextureData( ColorData );
                            ColorTexture.Apply();

                            depthFrame.CopyFrameDataToArray( DepthData );

                            depthFrame.Dispose();
                            depthFrame = null;

                            // Map depth to RGBD
                            AlignRGBD();

                            // Unproject to 3D points in camera space
                            PreparePointCloud();

                        }

                        colorFrame.Dispose();
                        colorFrame = null;
                    }

                    frame = null;
                }
            }

        }

		/// <summary>
		/// Given a color space point with x,y coordinates of a pixel location
		/// return the corresponding rgb pixel 
		/// </summary>
		/// <returns>The pixel value.</returns>
		/// <param name="pt">Point.</param>
		public Color GetPixelValue( ColorSpacePoint pt ) {
			return ColorTexture.GetPixel (pt.X, pt.Y);
		}

		/// <summary>
		/// Creates a lookup list ColorPoints that map DepthData to ColorData
		/// </summary>
        private void AlignRGBD()
        {
            // Stores RGB points
            _Mapper.MapDepthFrameToColorSpace( DepthData, ColorPoints );
        }

        // Gets 3D points [units = meters] and calculated pf vector
        private void PreparePointCloud()
        {
            boundingbox = findBoundingBoxFromCurrentState();

            // Stores 3D point cloud
            _Mapper.MapDepthFrameToCameraSpace( DepthData, Camera3DPoints );

            for (int i = 0; i < Height; i++)
            {
                for (int j = 0; j < Width; j++)
                {

                    int idx = i * Width + j;

                    // if point is not in the bounding box
                    if (j < boundingbox.x || j >= boundingbox.z || i < boundingbox.y || i >= boundingbox.w)
                    {
                        // mark as not useful
                        Camera3DPoints[idx].X = 0;
                        Camera3DPoints[idx].Y = 0;
                        Camera3DPoints[idx].Z = 0;
                        PfVec[idx] = -1;
                    }
                    else
                    {
                        // calculate pf
						PfVec[idx] = GetPf( GetPixelValue(ColorPoints[idx]) );
                    }
                }
            }
        }

		// TODO
		// Use the current bounding box to assign foreground pixels
		// Everything else assign as background
		// This updated the vector Mask
		public void LabelForegroundFromBoundingBox () {
			//Mask = //
		}

		// TODO if want to Downsample
        private void DownsampleImage()
        {
        }

        private void GetPf( Color pixel )
        {
            int noBins = histogram.BinsNumber;
            int ru = pixel.r / noBins;
            int gu = pixel.g / noBins;
            int bu = pixel.b / noBins;
            int pidx = ru * noBins * noBins + gu * noBins + bu;
            return histogram[pidx];
        }

        // TODO
        // Project current pose points and create a bounding box that we will use
		// for the histogram
		private void findBoundingBoxFromCurrentState()
        {
        }

        public void ComputePfImageFromHistogram()
        {

            int noBins = histogram.BinsNumber;
            float pf = 0;

            for (int i = 0; i < Height; i++)
            {
                for (int j = 0; j < Width; j++)
                {

                    int idx = i * Width + Height;
                    Color pixel = ColorTexture.GetPixel( ColorPoints[idx].X, ColorPoints[idx].Y );
                    pf = GetPf( pixel );
                    if (pf > 0.5f)
                    {
                        PfImage.SetPixel( j, i, Color.red );
                    }
                    else if (pf == 0.5f)
                    {
                        PfImage.SetPixel( j, i, Color.blue );
                    }
                }
            }
        }

        void OnApplicationQuit()
        {
            if (_Reader != null)
            {
                _Reader.Dispose();
                _Reader = null;
            }

            if (_Sensor != null)
            {
                if (_Sensor.IsOpen)
                {
                    _Sensor.Close();
                }

                _Sensor = null;
            }
        }

    }
}
