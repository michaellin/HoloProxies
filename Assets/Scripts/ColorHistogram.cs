using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Windows.Kinect;

using HoloProxies.Utils;

namespace HoloProxies.Objects
{
	
	public class ColorHistogram  {

		public Vector2[] data_unnormalized;
		public Vector2[] data_normalized;

		public float[] posterior;

		public int BinsNumber, dim;

		private int FrameWidth, FrameHeight;

		public ColorHistogram (int nbins, int width, int height ) {
			BinsNumber = nbins;
			dim = BinsNumber * BinsNumber * BinsNumber;

			data_normalized = new Vector2[dim];
			data_unnormalized = new Vector2[dim];

			posterior = new float[dim];

			FrameWidth = width;
			FrameHeight = height;
		}

        // Update histogram based on an existing histogram
        /// <summary>
        /// Function to update current HistogramRGB given a new HistogramRGB. However, update it
        /// at a decaying rate. Update the foreground at rf rate and background at rb rate.
        /// </summary>
        public void UpdateHistogram( ColorHistogram newHist, float rf, float rb ) {
			for (int i = 0; i < dim; i++) {
				this.data_normalized [i].x = this.data_normalized [i].x * (1 - rf) + newHist.data_normalized [i].x * rf;
				this.data_normalized[i].y = this.data_normalized[i].y * (1 - rb) + newHist.data_normalized[i].y * rb;
				this.posterior[i] = this.data_normalized[i].x / (this.data_normalized[i].x + this.data_normalized[i].y);
			}
		}

        /// <summary>
        /// Function to build a histogram from color and mask. These two are RGBA32 format.
        /// </summary>
		public void BuildHistogram( ColorSpacePoint[] color, Texture2D colorTex, char[] Mask ) {
			int ru, gu, bu;
			int pidx;

			float sumForeground = 0;
			float sumBackground = 0;


			for (int j = 0; j < FrameHeight; j++) {
				for (int i = 0; i < FrameWidth; i++) {
					int idx = i + j * FrameWidth;
					ColorSpacePoint pt = color [idx];
					Color pixel = colorTex.GetPixel ((int) pt.X, (int) pt.Y);
					ru = (int) (pixel.r / BinsNumber);
					gu = (int) (pixel.g / BinsNumber);
					bu = (int) (pixel.b / BinsNumber);
					pidx = ru * BinsNumber * BinsNumber + gu * BinsNumber + bu;
                    
                    char m = Mask[idx];
					if (m == defines.HIST_FG_PIXEL) { // foreground pixel 
						data_unnormalized [pidx].x++;
						sumForeground++;
					} else if (m == defines.HIST_BG_PIXEL) { // background 
						data_unnormalized [pidx].y++;
						sumBackground++;
					} else if (m == defines.HIST_USELESS_PIXEL) { // useless pixels
					} else { // any other color is far background
					}
						
				}
			}

			sumForeground = (sumForeground != 0) ? 1.0f/sumForeground : 0;
			sumBackground = (sumBackground != 0) ? 1.0f/sumBackground : 0;

			for ( int i=0; i<dim; i++) {
				data_normalized[i].x = data_unnormalized[i].x + sumForeground + 0.0001f;
				data_normalized[i].y = data_unnormalized[i].y + sumBackground + 0.0001f;
				posterior[i] = data_normalized[i].x / (data_normalized[i].x + data_normalized[i].y);
			}
		}

		// TODO - this method is deprecated
		// Since we won't follow the convention of labeling the RGBD image
		// Instead we will label the mask
        /// <summary>
        /// Function to build a histogram from RGBD. Input format is RGBAFloat.
        /// </summary>
		public void BuildHistogramFromLabeledRGBD( ColorSpacePoint[] color, Texture2D colorTex, ushort[] depth ) {
			int ru, gu, bu;
			int pidx;

			float sumForeground = 0;
			float sumBackground = 0;


			for (int j = 0; j < FrameHeight; j++) {
				for (int i = 0; i < FrameWidth; i++) {
					int idx = i + j * FrameWidth;


					ColorSpacePoint pt = color [idx];
					Color pixel = colorTex.GetPixel ((int) pt.X, (int) pt.Y);
					float w = depth[idx];

					if ( w >= defines.HIST_USELESS_PIXEL )
						continue;

                    ru = (int)(pixel.r / BinsNumber);
                    gu = (int)(pixel.g / BinsNumber);
                    bu = (int)(pixel.b / BinsNumber);
                    pidx = ru * BinsNumber * BinsNumber + gu * BinsNumber + bu;

					// TODO these colors can be replaced by macros?
                    // TODO check this cast to int
					switch ( (int)w ) {

					case defines.HIST_FG_PIXEL: // foreground pixel
						data_unnormalized [pidx].x++;
						sumForeground++;
						break;

					case defines.HIST_BG_PIXEL: // background pixel
						data_unnormalized [pidx].y++;
						sumBackground++;
						break;

					default: 
						break;

					}
				}
			}

			sumForeground = (sumForeground != 0) ? 1.0f/sumForeground : 0;
			sumBackground = (sumBackground != 0) ? 1.0f/sumBackground : 0;

			for ( int i=0; i<dim; i++) {
				data_normalized[i].x = data_unnormalized[i].x + sumForeground + 0.0001f;
				data_normalized[i].y = data_unnormalized[i].y + sumBackground + 0.0001f;
				posterior[i] = data_normalized[i].x / (data_normalized[i].x + data_normalized[i].y);
			}
		}

		// TODO - deprecated, should be using mask instead
        /// <summary>
        /// Function to update a histogram from RGBD at rates rf and rb. This can call buildHistogramFromLabelledRGBD
        /// and updateHistogram. Input format is RGBAFloat.
        /// </summary>
        /// <param name="rf"></param>
        /// <param name="rb"></param>
		public void UpdateHistogramFromLabeledRGBD(float rf, float rb, ColorSpacePoint[] color, Texture2D colorTex, ushort[] DepthData)
		{
			ColorHistogram tmpHist = new ColorHistogram(this.BinsNumber, this.FrameWidth, this.FrameHeight);
			this.BuildHistogramFromLabeledRGBD (color, colorTex, DepthData);
			this.UpdateHistogram (tmpHist, rf, rb);
		}

		/// <summary>
		/// Function to update a histogram from RGBD at rates rf and rb. 
		/// The labeling is done on the frame Mask before this function call.
		/// </summary>
		/// <param name="rf"></param>
		/// <param name="rb"></param>
		public void UpdateHistogramFromLabeledMask(float rf, float rb, ColorSpacePoint[] color, Texture2D colorTex, short[] Mask)
		{
			ColorHistogram tmpHist = new ColorHistogram(this.BinsNumber, this.FrameWidth, this.FrameHeight);
			this.BuildHistogram (color, colorTex, Mask);
			this.UpdateHistogram (tmpHist, rf, rb);
		}

		// TODO do we need bb?
		public void buildHistogramFromLabeledRGBD()
		{
		}

		public void updateHistogramFromLabeledRGBD(float rf, float rb)
		{
		}


	}

}
