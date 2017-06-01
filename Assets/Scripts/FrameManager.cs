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
        public int height;
        public int width;

        public Vector2i depth_size;
        public Vector2i rgb_size;

        //TODO ISRView* view; Do we need this?

        public histogramRGB hist;

        public Vector4 boundingbox;

		public int DownsampleSize = 2;

		public int FrameWidth { get; private set; }
		public int FrameHeight { get; private set; }

		public int ColorWidth { get; private set; }
		public int ColorHeight { get; private set; }

		public int DepthWidth { get; private set; }
		public int DepthHeight { get; private set; }

		public CameraSpacePoint[] Camera3DPoints { get; private set; }
		public float[] PfVec { get; private set; }
		public Texture2D PfImage{ get; private set; }

		public Texture2D ColorTexture { get; private set; }
		public ColorSpacePoint[] ColorPoints { get; private set; }
		public byte[] ColorData { get; private set; }

		public ushort[] DepthData { get; private set; }

		private KinectSensor _Sensor;
		private MultiSourceFrameReader _Reader;
		private CoordinateMapper _Mapper;

		// Constructor
        public FrameManager( )
        {
			_Sensor = KinectSensor.GetDefault();

			if (_Sensor != null) 
			{
				_Mapper = _Sensor.CoordinateMapper;

				_Reader = _Sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth);

				var colorFrameDesc = _Sensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Rgba);
				ColorWidth = colorFrameDesc.Width;
				ColorHeight = colorFrameDesc.Height;
				ColorData = new byte[colorFrameDesc.BytesPerPixel * colorFrameDesc.LengthInPixels];
				ColorTexture = new Texture2D(ColorWidth, ColorHeight, TextureFormat.RGBA32, false);

				var depthFrameDesc = _Sensor.DepthFrameSource.FrameDescription;
				DepthWidth = depthFrameDesc.Width;
				DepthHeight = depthFrameDesc.Height;
				DepthData = new ushort[depthFrameDesc.LengthInPixels];

				// TODO
				// Downsample all data frames if necessary

				// Set FrameWidth and FrameHeight to the downsampled size
				FrameWidth = DepthWidth;
				FrameHeight = DepthHeight;

				PfImage = new Texture2D(FrameWidth, FrameHeight, TextureFormat.RGBA32, false);

				// Set buffers to align depth data to RGB and to align camera points
				ColorPoints = new ColorSpacePoint[DepthWidth * DepthHeight];
				Camera3DPoints = new CameraSpacePoint[DepthWidth * DepthHeight];
				PfVec = new float[FrameWidth * FrameHeight];

				if (!_Sensor.IsOpen)
				{
					_Sensor.Open();
				}
			}
        }

		public void UpdateFrame() {
			
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
							colorFrame.CopyConvertedFrameDataToArray(ColorData, ColorImageFormat.Rgba);
							ColorTexture.LoadRawTextureData(ColorData);
							ColorTexture.Apply();

							depthFrame.CopyFrameDataToArray(DepthData);

							depthFrame.Dispose();
							depthFrame = null;

							// Map depth to RGBD
							AlignRGBD ();

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

		// Creates a lookup list ColorPoints that map DepthData to ColorData
		private void AlignRGBD () {
			// Stores RGB points
			_Mapper.MapDepthFrameToColorSpace(DepthData, ColorPoints);
		}

		// Gets 3D points [units = meters] and calculated pf vector
		private void PreparePointCloud () {
			boundingbox = findBoundingBoxFromCurrentState ();

			// Stores 3D point cloud
			_Mapper.MapDepthFrameToCameraSpace(DepthData, Camera3DPoints);

			for (int i = 0; i < FrameHeight; i++) {
				for (int j = 0; j < FrameWidth; j++) {

					int idx = i * FrameWidth + j;

					// if point is not in the bounding box
					if (j < boundingbox.x || j >= boundingbox.z || i < boundingbox.y || i >= boundingbox.w) {
						// mark as not useful
						Camera3DPoints [idx].X = 0;
						Camera3DPoints [idx].Y = 0;
						Camera3DPoints [idx].Z = 0;
						PfVec [idx] = -1;
					} else {
						Color pixel = ColorTexture.GetPixel (ColorPoints [idx].X, ColorPoints [idx].Y);
						// calculate pf
						PfVec [idx] = GetPf ( pixel );
					}
				}
			}
		}

		private void DownsampleImage () {
		}

		private void GetPf( Color pixel) {
			int noBins = ColorHistogram.BinsNumber;
			int ru = pixel.r / noBins;
			int gu = pixel.g / noBins;
			int bu = pixel.b / noBins;
			int pidx = ru * noBins * noBins + gu * noBins + bu;
			return ColorHistogram [pidx];
		}


		// TODO
		private void findBoundingBoxFromCurrentState () {
		}

		public void ComputePfImageFromHistogram () {

			int noBins = ColorHistogram.BinsNumber;
			float pf = 0;

			for (int i = 0; i < FrameHeight; i++) {
				for (int j = 0; j < FrameWidth; j++) {

					int idx = i * FrameWidth + FrameHeight;
					Color pixel = ColorTexture.GetPixel (ColorPoints [idx].X, ColorPoints [idx].Y);
					pf = GetPf (pixel);
					if (pf > 0.5f) {
						PfImage.SetPixel (j, i, Color.red);
					} else if (pf == 0.5f) {
						PfImage.SetPixel (j, i, Color.blue);
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
