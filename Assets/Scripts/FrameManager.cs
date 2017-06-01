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
        public Vector4 boundingbox;

		public int DownsampleSize = 2;

		public int Width { get; private set; }
		public int Height { get; private set; }

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
				Width = DepthWidth;
				Height = DepthHeight;

				PfImage = new Texture2D(Width, Height, TextureFormat.RGBA32, false);

				// Set buffers to align depth data to RGB and to align camera points
				ColorPoints = new ColorSpacePoint[DepthWidth * DepthHeight];
				Camera3DPoints = new CameraSpacePoint[DepthWidth * DepthHeight];
				PfVec = new float[Width * Height];

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

			for (int i = 0; i < Height; i++) {
				for (int j = 0; j < Width; j++) {

					int idx = i * Width + j;

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
			int noBins = histogram.BinsNumber;
			int ru = pixel.r / noBins;
			int gu = pixel.g / noBins;
			int bu = pixel.b / noBins;
			int pidx = ru * noBins * noBins + gu * noBins + bu;
			return histogram [pidx];
		}


		// TODO
		private void findBoundingBoxFromCurrentState () {
//			Vector3f crns[8];
//			Vector3f *ipts = new Vector3f[state->numPoses() * 8];
//			Vector4i bb(imgsize.x,imgsize.y,0,0);
//
//			for (int i = -1, idx = 0; i <= 1; i += 2)
//				for (int j = -1; j <= 1; j += 2)
//					for (int k = -1; k <= 1; k += 2, idx++) 
//						crns[idx] = Vector3f(i*0.1f, j*0.1f, k*0.1f);
//
//			for (int i = 0, idx=0; i < state->numPoses(); i++) for (int j = 0; j < 8;j++, idx++)
//			{
//				Matrix4f H = state->getPose(i)->getH();
//				ipts[idx] = K*(H*crns[j]);		ipts[idx].x /= ipts[idx].z;		ipts[idx].y /= ipts[idx].z;
//
//				bb.x = ipts[idx].x < bb.x ? ipts[idx].x : bb.x;
//				bb.y = ipts[idx].y < bb.y ? ipts[idx].y : bb.y;
//				bb.z = ipts[idx].x > bb.z ? ipts[idx].x : bb.z;
//				bb.w = ipts[idx].y > bb.w ? ipts[idx].y : bb.w;
//			}
//
//			bb.x = bb.x < 0 ? 0 : bb.x; 
//			bb.y = bb.y < 0 ? 0 : bb.y;
//			bb.z = bb.z > imgsize.x ? imgsize.x : bb.z;
//			bb.w = bb.w > imgsize.y ? imgsize.y : bb.w;
//
//			return bb;
		}

		public void ComputePfImageFromHistogram () {

			int noBins = histogram.BinsNumber;
			float pf = 0;

			for (int i = 0; i < Height; i++) {
				for (int j = 0; j < Width; j++) {

					int idx = i * Width + Height;
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
