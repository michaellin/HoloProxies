﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Windows.Kinect;

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

        public CameraSpacePoint[] Camera3DPoints_full { get; private set; }
        public CameraSpacePoint[] Camera3DPoints { get; private set; }
        public float[] PfVec { get; private set; }
        public Texture2D PfImage { get; private set; }

        public Texture2D ColorTexture_full { get; private set; }
        public Texture2D ColorTexture { get; private set; }
        public Texture2D ColorTextureVisual { get; private set; }
        public ColorSpacePoint[] ColorPoints_full { get; private set; }
        //public ColorSpacePoint[] ColorPoints { get; private set; }
        public byte[] ColorData_full { get; private set; }
        public byte[] ColorData { get; private set; }

        public short[] Mask { get; private set; }

        public ushort[] DepthData_full { get; private set; }
        public ushort[] DepthData { get; private set; }
        public Texture2D DepthTexture_full { get; private set; }
        private byte[] DepthRaw;

        public Matrix4x4 K;

        // histogram used to keep track of the probability distribution of pixels
        public ColorHistogram histogram;

        private KinectSensor _Sensor;
        private MultiSourceFrameReader _Reader;
        private CoordinateMapper _Mapper;

        private bool drawBox = true;
        private bool plotHistogram = false;

        public Texture2D HistogramTexture;

        // Constructor
        public FrameManager()
        {
            _Sensor = KinectSensor.GetDefault();

            if (_Sensor != null)
            {

                _Mapper = _Sensor.CoordinateMapper;

                _Reader = _Sensor.OpenMultiSourceFrameReader( FrameSourceTypes.Color | FrameSourceTypes.Depth );

                var depthFrameDesc = _Sensor.DepthFrameSource.FrameDescription;
                DepthWidth = depthFrameDesc.Width;
                DepthHeight = depthFrameDesc.Height;
                DepthData_full = new ushort[depthFrameDesc.LengthInPixels];
                DepthData = new ushort[depthFrameDesc.LengthInPixels / (defines.DOWNSAMPLE * defines.DOWNSAMPLE)];
                //DepthRaw = new byte[depthFrameDesc.LengthInPixels * 4];
                //DepthTexture_full = new Texture2D( DepthWidth, DepthHeight, TextureFormat.RGBA32, false );

                // Set FrameWidth and FrameHeight to the downsampled size
                Width = DepthWidth / defines.DOWNSAMPLE;
                Height = DepthHeight / defines.DOWNSAMPLE;

                var colorFrameDesc = _Sensor.ColorFrameSource.CreateFrameDescription( ColorImageFormat.Rgba );
                ColorWidth = colorFrameDesc.Width;
                ColorHeight = colorFrameDesc.Height;
                ColorData_full = new byte[colorFrameDesc.BytesPerPixel * colorFrameDesc.LengthInPixels];
                ColorData = new byte[colorFrameDesc.BytesPerPixel * Width * Height];
                ColorTexture_full = new Texture2D( ColorWidth, ColorHeight, TextureFormat.RGBA32, false );
                ColorTexture = new Texture2D( Width, Height, TextureFormat.RGBA32, false );
                ColorTextureVisual = new Texture2D( Width, Height, TextureFormat.RGBA32, false );

                //PfImage = new Texture2D( Width, Height, TextureFormat.RGBA32, false );

                // Set buffers to align depth data to RGB and to align camera points
                ColorPoints_full = new ColorSpacePoint[depthFrameDesc.LengthInPixels];
                Camera3DPoints_full = new CameraSpacePoint[DepthWidth * DepthHeight];
                Camera3DPoints = new CameraSpacePoint[Width * Height];
                PfVec = new float[Width * Height];
                Mask = new short[Width * Height];
                
                // Save camera intrinsics matrix
                CameraIntrinsics intrinsics = _Mapper.GetDepthCameraIntrinsics();
                K = new Matrix4x4();
                K.m00 = intrinsics.FocalLengthX / defines.DOWNSAMPLE;
                K.m01 = 0;
                K.m02 = intrinsics.PrincipalPointX / defines.DOWNSAMPLE;
                K.m10 = 0;
                K.m11 = intrinsics.FocalLengthY / defines.DOWNSAMPLE;
                K.m12 = intrinsics.PrincipalPointY / defines.DOWNSAMPLE;
                K.m20 = 0; K.m21 = 0;
                K.m22 = 1.0f;// / defines.DOWNSAMPLE;
               
                // TODO print K
                Debug.Log( "K = " + K );
                Debug.Log( "Color Width: " + ColorWidth + " Color Height: " + ColorHeight );
                Debug.Log( "Depth Width: " + DepthWidth + " Depth Height: " + DepthHeight );
                Debug.Log( " Width: " + Width + " Height: " + Height );

                // Initialize color histogram
                histogram = new ColorHistogram( defines.HISTOGRAM_NBIN, Width, Height );


                HistogramTexture = new Texture2D( 4, 4, TextureFormat.RFloat, false );
                //Debug.Log( HistogramTexture.width );
                //Debug.Log( HistogramTexture.height );
                //Debug.Log( histogram.posterior.Length );
                
                if (!_Sensor.IsOpen)
                {
                    _Sensor.Open();
                }
            }
        }

        /// <summary>
        /// Updates the frame by grabbing the rgb and depth from the Kinnect.
        /// </summary>
        /// <returns><c>true</c>, if frame was successfully updated, <c>false</c> otherwise.</returns>
        /// <param name="state">State.</param>
        public bool UpdateFrame( HoloProxies.Engine.trackerState state )
        {
            bool success = false;

            if (_Reader != null)
            {
                var frame = _Reader.AcquireLatestFrame();
                if (frame != null)
                {
                    //Debug.Log( "in here1" );
                    var colorFrame = frame.ColorFrameReference.AcquireFrame();
                    if (colorFrame != null)
                    {
                        //Debug.Log( "in here2" );
                        var depthFrame = frame.DepthFrameReference.AcquireFrame();
                        if (depthFrame != null)
                        {
                            //Debug.Log( "in here3" );
                            // get color data + texture
                            colorFrame.CopyConvertedFrameDataToArray( ColorData_full, ColorImageFormat.Rgba );

                            // get depth data
                            depthFrame.CopyFrameDataToArray( DepthData_full );

                            // Create a depth texture
                            //int index = 0;
                            //foreach (var ir in DepthData_full)
                            //{
                            //    byte intensity = (byte)(ir >> 8);
                            //    DepthRaw[index++] = intensity;
                            //    DepthRaw[index++] = intensity;
                            //    DepthRaw[index++] = intensity;
                            //    DepthRaw[index++] = 255; // Alpha
                            //}
                            //DepthTexture_full.LoadRawTextureData( DepthRaw );
                            //DepthTexture_full.Apply(); TODO fix this

                            // dispose frame
                            depthFrame.Dispose();
                            depthFrame = null;

                            // Map depth to RGBD
                            AlignRGBD();

                            // Unproject to 3D points in camera space (based on bounding box)
                            UnprojectFrom2Dto3D();

                            // Downsample all data frames if necessary
                            DownsampleAndFilterRGBDImage();

                            // Apply downsampled texture
                            ColorTexture.LoadRawTextureData( ColorData );
                            ColorTextureVisual.LoadRawTextureData( ColorData );

                            // Unproject points to 3D space
                            PreparePointCloud( state );

                            if (plotHistogram)
                            {
                                for (int i = 0; i < HistogramTexture.height; i++)
                                {
                                    for (int j = 0; j < HistogramTexture.width; j++)
                                    {
                                        int idx = i * HistogramTexture.width + j;
                                        HistogramTexture.SetPixel( j, i, new Color( histogram.posterior[idx * 256], 0, 0 ) );
                                    }
                                }
                                HistogramTexture.Apply();
                            }

                            success = true;
                        }
                        colorFrame.Dispose();
                        colorFrame = null;
                    }
                    frame = null;
                }
            }
            return success;
        }

        /// <summary>
        /// Given a color space point with x,y coordinates of a pixel location
        /// return the corresponding rgb pixel 
        /// </summary>
        /// <returns>The pixel value.</returns>
        /// <param name="pt">Point.</param>
        public Color GetPixelValue( ColorSpacePoint pt )
        {
            return ColorTexture.GetPixel( (int)pt.X, (int)pt.Y );
        }

        /// <summary>
        /// Given a color space point with x,y coordinates of a pixel location
        /// set the corresponding pixel to the given color
        /// </summary>
        /// <param name="pt">Point.</param>
        public void SetPixelValue( ColorSpacePoint pt, Color color )
        {
            ColorTexture.SetPixel( (int)pt.X, (int)pt.Y, color );
        }

        /// <summary>
        /// Creates a lookup list ColorPoints that map DepthData to ColorData
        /// </summary>
        private void AlignRGBD()
        {
            // Stores RGB points
            _Mapper.MapDepthFrameToColorSpace( DepthData_full, ColorPoints_full );
        }

        /// <summary>
        /// Gets 3D points [units = meters] and calculated pf vector
        /// </summary>
        /// <param name="state">State.</param>
        private void UnprojectFrom2Dto3D()
        {
            // Stores 3D point cloud
            _Mapper.MapDepthFrameToCameraSpace( DepthData_full, Camera3DPoints_full );
        }

        /// <summary>
        /// Gets 3D points [units = meters] and calculated pf vector
        /// </summary>
        /// <param name="state">State.</param>
        private void PreparePointCloud( HoloProxies.Engine.trackerState state )
        {
            boundingbox = findBoundingBoxFromCurrentState( state );

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
                        PfVec[idx] = defines.OUTSIDE_BB;
                    }
                    else // if it's inside the box
                    {
                        Color pixel = ColorTexture.GetPixel( j, i );
                        PfVec[idx] = GetPf( pixel );
                        // Draw the bounding box region
                        if (drawBox && OnBoxEdge(j, i) )
                        {
                            //ColorTextureVisual.SetPixel( j, i, Color.blue );
                            ColorTextureVisual.SetPixel( j, i, Color.Lerp( pixel, Color.blue, 0.3f ) );
                        }
                    } 
                }
            } //end for
        }

        /// <summary>
        /// Returns true if index (i,j) lies within an inner frame of the bounding box.
        /// The frame width is set by defines.BB_MARGIN.
        /// </summary>
        /// <param name="j"></param>
        /// <param name="i"></param>
        /// <returns></returns>
        private bool OnBoxEdge( int j, int i )
        {
            // if not on inner region 
            if ((j < boundingbox.x + defines.BB_MARGIN) || (j >= boundingbox.z - defines.BB_MARGIN)
                || (i < boundingbox.y + defines.BB_MARGIN) || (i >= boundingbox.w - defines.BB_MARGIN)) {
                return true;
            }

            return false;
        }
        

        /// <summary>
        /// Re-initializes the histogram based on current state. 
        /// Based on the pose, projects a foreground bounding box (everything 
        /// else is background) and stores it in Mask. Only the foreground pixels 
        /// are used to create the histogram.
        /// </summary>
        /// <param name="state">State.</param>
        public void ReinitHistogramFromRendering( HoloProxies.Engine.trackerState state )
        {
            LabelMaskFromCurrentStateBoundingBox( state );
            histogram.BuildHistogram( ColorTexture, Mask );
        }

        /// <summary>
        /// Downsamples the image.
        /// </summary>
        private void DownsampleAndFilterRGBDImage()
        {
            for (int y = 0; y < Height; y++)
            {
                for (int x = 0; x < Width; x++)
                {
                    filterDownsampleWithHoles( x, y ); 
                }
            }
        }


        private void filterDownsampleWithHoles( int currX, int currY )
        {
            int inPosX = currX * defines.DOWNSAMPLE;
            int inPosY = currY * defines.DOWNSAMPLE;

            ushort depthPixelIn;
            ushort goodPixelN = 0;
            ColorSpacePoint colorPt;
            ushort[] tempColor = new ushort[3];
            tempColor[0] = 0; tempColor[1] = 0; tempColor[2] = 0;
            bool badPixel = false; // TODO
            // Downsample point cloud
            Camera3DPoints[currX + currY * Width] = Camera3DPoints_full[inPosX + inPosY * DepthWidth];

            for (int x = 0; x < defines.DOWNSAMPLE; x++)
            {
                for (int y = 0; y < defines.DOWNSAMPLE; y++)
                {
                    depthPixelIn = DepthData_full[(inPosX + x) + (inPosY + y) * DepthWidth];
                    colorPt = ColorPoints_full[(inPosX + x) + (inPosY + y) * DepthWidth];

                    if ((depthPixelIn > 0) && !float.IsInfinity( colorPt.X ) && !float.IsInfinity( colorPt.Y )
                        && (colorPt.X > 0) && (colorPt.Y > 0)
                        && (colorPt.X < ColorWidth) && (colorPt.Y < ColorHeight)) // only take positive depth and non-infinity values
                    {
                        DepthData[currX + currY * Width] += depthPixelIn;
                        goodPixelN++;

                        tempColor[0] += ColorData_full[((int)colorPt.X + (int)colorPt.Y * ColorWidth) * 4];
                        tempColor[1] += ColorData_full[((int)colorPt.X + (int)colorPt.Y * ColorWidth) * 4 + 1];
                        tempColor[2] += ColorData_full[((int)colorPt.X + (int)colorPt.Y * ColorWidth) * 4 + 2];
                    } else
                    {
                        badPixel = true; // TODO
                    }
                }
            }

            
            if (!badPixel) //( goodPixelN > 0)
            {
                ColorData[(currX + currY * Width) * 4] = (byte)(tempColor[0] / goodPixelN);
                ColorData[(currX + currY * Width) * 4 + 1] = (byte)(tempColor[1] / goodPixelN);
                ColorData[(currX + currY * Width) * 4 + 2] = (byte)(tempColor[2] / goodPixelN);
                ColorData[(currX + currY * Width) * 4 + 3] = 255;

                DepthData[currX + currY * Width] /= goodPixelN;

                //TODO delete
                colorPt = ColorPoints_full[(inPosX) + (inPosY) * DepthWidth];
                if (!float.IsInfinity( colorPt.X ) && !float.IsInfinity( colorPt.Y )
                        && (colorPt.X > 0) && (colorPt.Y > 0)
                        && (colorPt.X < ColorWidth) && (colorPt.Y < ColorHeight))
                {
                    ColorData[(currX + currY * Width) * 4] = ColorData_full[((int)colorPt.X + (int)colorPt.Y * ColorWidth) * 4];
                    ColorData[(currX + currY * Width) * 4 + 1] = ColorData_full[((int)colorPt.X + (int)colorPt.Y * ColorWidth) * 4 + 1];
                    ColorData[(currX + currY * Width) * 4 + 2] = ColorData_full[((int)colorPt.X + (int)colorPt.Y * ColorWidth) * 4 + 2];
                }

                if ( Mask[currX + currY * Width] == defines.HIST_USELESS_PIXEL )
                {
                    Mask[currX + currY * Width] = 0;
                } 
            }
            else
            {
                ColorData[(currX + currY * Width) * 4] = 0;
                ColorData[(currX + currY * Width) * 4 + 1] = 0;
                ColorData[(currX + currY * Width) * 4 + 2] = 0;
                ColorData[(currX + currY * Width) * 4 + 3] = 255;
                Mask[currX + currY * Width] = defines.HIST_USELESS_PIXEL;
            }

        }

        private Color GetColorFromData( int idx )
        {
            return new Color( ColorData[idx * 4], ColorData[idx * 4 + 1], ColorData[idx * 4 + 2], ColorData[idx * 4 + 3] );
        }

        /// <summary>
        /// Gets the pf.
        /// </summary>
        /// <param name="pixel">Pixel.</param>
        private float GetPf( Color pixel )
        {
            int ru, gu, bu;
            int noBins = histogram.BinsNumber;
            //ru = (int)((pixel.r) / ((float)noBins));
            //gu = (int)((pixel.g) / ((float)noBins));
            //bu = (int)((pixel.b) / ((float)noBins));
            ru = (int)((pixel.r * 255.0f) / noBins);
            gu = (int)((pixel.g * 255.0f) / noBins);
            bu = (int)((pixel.b * 255.0f) / noBins);
            int pidx = ru * noBins * noBins + gu * noBins + bu;

            return histogram.posterior[pidx];
        }

        /// <summary>
        /// Use the current bounding box to assign foreground pixels
        /// Everything else assign as background
        /// This updates the vector Mask
        /// </summary>
        private void LabelMaskFromCurrentStateBoundingBox( HoloProxies.Engine.trackerState state )
        {
       
            // first get bounding box for foreground pixels
            boundingbox = findBoundingBoxFromCurrentState( state );

            //  (x,y) ----- +
            //    |         |
            //    |         |
            //    + ----- (z,w)
            // set the near background bounding box
            UnityEngine.Vector4 boundingboxBG =
                new UnityEngine.Vector4( boundingbox.x - defines.BB_MARGIN, //top, left
                    boundingbox.y - defines.BB_MARGIN,
                    boundingbox.z + defines.BB_MARGIN, //bottom, right
                    boundingbox.w + defines.BB_MARGIN );

            for (int i = 0; i < Height; i++)
            {
                for (int j = 0; j < Width; j++)
                {
                    int idx = i * Width + j;
                    // if it's not a zero depth pixel i.e. useless
                    //if (Mask[idx] != defines.HIST_USELESS_PIXEL)
                    //{
                    // if point is in the foreground bounding box
                    if (j > boundingbox.x && j < boundingbox.z && i > boundingbox.y && i < boundingbox.w)
                    {
                        Mask[idx] = defines.HIST_FG_PIXEL;
                    }
                    // else if it is in the near background
                    else if (j > boundingboxBG.x && j < boundingboxBG.z && i > boundingboxBG.y && i < boundingboxBG.w)
                    {
                        Mask[idx] = defines.HIST_BG_PIXEL;
                    } else // else it is in the far background
                    {
                        Mask[idx] = defines.HIST_USELESS_PIXEL;
                    }
                    //} 
                }
            } //end for
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
            Vector3[] ipts = new Vector3[state.numPoses() * 8];
            UnityEngine.Vector4 bb = new UnityEngine.Vector4( Width, Height, 0, 0 );
            for (int i = -1, idx = 0; i <= 1; i += 2)
            {
                for (int j = -1; j <= 1; j += 2)
                {
                    for (int k = -1; k <= 1; k += 2, idx++)
                    {
                        corners[idx] = new Vector3( i * 0.075f, j * 0.075f, k * 0.075f );
                    }
                }
            }

            for (int i = 0, idx = 0; i < state.numPoses(); i++)
            {
                for (int j = 0; j < 8; j++, idx++)
                {
                    Matrix4x4 H = state.getPose( i ).getH();
                    Vector3 temp = H.MultiplyPoint( corners[j] );
                    ipts[idx] = K * (H.MultiplyPoint( corners[j] ));
                    ipts[idx].x /= ipts[idx].z; ipts[idx].y /= ipts[idx].z;

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
            float pf = 0;
            for (int i = 0; i < Height; i++)
            {
                for (int j = 0; j < Width; j++)
                {
                    int idx = i * Width + j;
                    Color pixel = ColorTexture.GetPixel( j, i );
                    pf = GetPf( pixel );
                    if (pf > 0.8f)
                    {
                        if (!ColorTextureVisual.GetPixel( j, i ).Equals(Color.blue)) //&& Mask[idx] == defines.HIST_FG_PIXEL)
                        {
                            ColorTextureVisual.SetPixel( j, i, Color.Lerp( pixel, Color.green, 0.5f ) );

                            if ( Mask[idx] == defines.HIST_FG_PIXEL)
                            {
                                ColorTextureVisual.SetPixel( j, i, Color.Lerp( pixel, Color.cyan, 0.5f ) );
                            }
                        }
                    }
                    else if (pf == 0.3f)
                    {
                        if (!ColorTextureVisual.GetPixel( j, i ).Equals( Color.blue ))
                        {
                            //ColorTextureVisual.SetPixel( j, i, Color.Lerp( pixel, Color.yellow, 0.2f ) );
                        }
                    }
                }
            }
            ColorTextureVisual.Apply();
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
