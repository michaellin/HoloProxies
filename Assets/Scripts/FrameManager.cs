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

		public short[] Mask { get; private set; }

        public ushort[] DepthData { get; private set; }

		public Matrix4x4 K;

        // histogram used to keep track of the probability distribution of pixels
        public ColorHistogram histogram;

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

                _Reader = _Sensor.OpenMultiSourceFrameReader( FrameSourceTypes.Color | FrameSourceTypes.Depth );

                var colorFrameDesc = _Sensor.ColorFrameSource.CreateFrameDescription( ColorImageFormat.Rgba );
                ColorWidth = colorFrameDesc.Width;
                ColorHeight = colorFrameDesc.Height;
                ColorData = new byte[colorFrameDesc.BytesPerPixel * colorFrameDesc.LengthInPixels];
                ColorTexture = new Texture2D( ColorWidth, ColorHeight, TextureFormat.RGBA32, false );

                var depthFrameDesc = _Sensor.DepthFrameSource.FrameDescription;
                DepthWidth = depthFrameDesc.Width;
                DepthHeight = depthFrameDesc.Height;
                DepthData = new short[depthFrameDesc.LengthInPixels];

                // TODO
                // Downsample all data frames if necessary
				SubsampleAndFilterRGBDImage();

                // Set FrameWidth and FrameHeight to the downsampled size
                Width = DepthWidth;
                Height = DepthHeight;

                PfImage = new Texture2D( Width, Height, TextureFormat.RGBA32, false );

                // Set buffers to align depth data to RGB and to align camera points
                ColorPoints = new ColorSpacePoint[DepthWidth * DepthHeight];
                Camera3DPoints = new CameraSpacePoint[DepthWidth * DepthHeight];
                PfVec = new float[Width * Height];
				Mask = new short[Width * Height];

                // Save camera intrinsics matrix
				CameraIntrinsics intrinsics = _Mapper.GetDepthCameraIntrinsics ();
				K = new Matrix4x4 ();
				K.m00 = intrinsics.FocalLengthX;    K.m01 = 0;                          K.m02 = intrinsics.PrincipalPointX;
				K.m10 = 0;                          K.m11 = intrinsics.FocalLengthY;    K.m12 = intrinsics.PrincipalPointY;
				K.m20 = 0;                          K.m21 = 0;                          K.m23 = 1;

                // Initialize color histogram
                histogram = new ColorHistogram( defines.HISTOGRAM_NBIN, Width, Height );

                if (!_Sensor.IsOpen)
                {
                    _Sensor.Open();
                }
            }

        }

		public void UpdateFrame(HoloProxies.Engine.trackerState state)
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

							// Unproject to 3D points in camera space (based on bounding box)
                            PreparePointCloud(state);
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
			return ColorTexture.GetPixel ((int) pt.X, (int) pt.Y);
		}

		/// <summary>
		/// Creates a lookup list ColorPoints that map DepthData to ColorData
		/// </summary>
        private void AlignRGBD()
        {
            // Stores RGB points
            _Mapper.MapDepthFrameToColorSpace( DepthData, ColorPoints );
        }

		/// <summary>
		/// Gets 3D points [units = meters] and calculated pf vector
		/// </summary>
		/// <param name="state">State.</param>
		private void PreparePointCloud( HoloProxies.Engine.trackerState state )
        {
			boundingbox = findBoundingBoxFromCurrentState ( state );

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
            } //end for
        }

		// TODO
		/// <summary>
		/// Use the current bounding box to assign foreground pixels
		/// Everything else assign as background
		/// This updates the vector Mask
		/// </summary>
		public void LabelForegroundFromStateBoundingBox ( HoloProxies.Engine.trackerState state, int bbmargin ) 
		{
			// first get bounding box for foreground pixels
			findBoundingBoxFromCurrentState (state);

			// set the near background bounding box
			UnityEngine.Vector4 boundingboxBG = new UnityEngine.Vector4(boundingbox.x, boundingbox.y, boundingbox.z, boundingbox.w);
			boundingboxBG += bbmargin;

			for (int i = 0; i < Height; i++)
			{
				for (int j = 0; j < Width; j++)
				{
					int idx = i * Width + j;
					// if point is in the foreground bounding box
					if (j > boundingbox.x || j < boundingbox.z || i > boundingbox.y || i < boundingbox.w)
					{
						Mask [idx] = defines.HIST_FG_PIXEL;
					}
					// else if it is in the near background
					else if (j > boundingboxBG.x || j < boundingboxBG.z || i > boundingboxBG.y || i < boundingboxBG.w)
					{
						Mask [idx] = defines.HIST_BG_PIXEL;
					}
				}
			} //end for
		}

		// TODO do we want to Downsample?
		/// <summary>
		/// Downsamples the image.
		/// </summary>
        private void SubsampleAndFilterRGBDImage()
        {
			// filter for zero depth pixels
			for (int i = 0; i < (Width * Height); i++) {
				if (DepthData [i] <= 0) {
					DepthData [i] = defines.HIST_USELESS_PIXEL;
				}
			}
        }

		/// <summary>
		/// Gets the pf.
		/// </summary>
		/// <param name="pixel">Pixel.</param>
        private float GetPf( Color pixel )
        {
            int ru, gu, bu;
            int noBins = histogram.BinsNumber;
            ru = (int)(pixel.r / noBins);
            gu = (int)(pixel.g / noBins);
            bu = (int)(pixel.b / noBins);
            int pidx = ru * noBins * noBins + gu * noBins + bu;
            return histogram.posterior[pidx];
        }

		/// <summary>
		/// Finds the state of the bounding box from current.
		/// Project current pose points and create a bounding box that we will use 
		/// for the histogram
		/// </summary>
		/// <param name="state">State.</param>
		private UnityEngine.Vector4 findBoundingBoxFromCurrentState( HoloProxies.Engine.trackerState state )
        {
			Vector3[] corners = new Vector3[8];
			Vector3[] ipts = new Vector3[state.numPoses () * 8];
			UnityEngine.Vector4 bb = new UnityEngine.Vector4(Width, Height, 0, 0);
			for (int i = -1, idx = 0; i <= 1; i += 2) {
				for (int j = -1; j <= 1; j += 2) {
					for (int k = -1; k <= 1; k += 2, idx++) {
						corners [idx] = new Vector3 (i * 0.1f, j * 0.1f, k * 0.1f);
					}
				}
			}

			for (int i = 0, idx = 0; i < state.numPoses (); i++) {
				for (int j = 0; j < 8; j++, idx++) {
					Matrix4x4 H = state.getPose (i).getH ();
					ipts [idx] = K * (H * corners [j]);
					ipts [idx].x /= ipts [idx].z;   ipts [idx].y /= ipts [idx].z;

					bb.x = ipts[idx].x < bb.x ? ipts[idx].x : bb.x;
					bb.y = ipts[idx].y < bb.y ? ipts[idx].y : bb.y;
					bb.z = ipts[idx].x > bb.z ? ipts[idx].x : bb.z;
					bb.w = ipts[idx].y > bb.w ? ipts[idx].y : bb.w;
				}
			}

			bb.x = bb.x < 0 ? 0 : bb.x; 
			bb.y = bb.y < 0 ? 0 : bb.y;
			bb.z = bb.z > Width ? Width : bb.z;
			bb.w = bb.w > Height ? Height : bb.w;

			return bb;
        }

		/// <summary>
		/// Computes the pf image from histogram. Used for visualization.
		/// </summary>
        public void ComputePfImageFromHistogram()
        {
            int noBins = histogram.BinsNumber;
            float pf = 0;

            for (int i = 0; i < Height; i++)
            {
                for (int j = 0; j < Width; j++)
                {

                    int idx = i * Width + Height;
                    Color pixel = ColorTexture.GetPixel( (int) ColorPoints[idx].X, (int) ColorPoints[idx].Y );
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
