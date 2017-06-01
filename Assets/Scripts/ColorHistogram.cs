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

		private FrameManager frame;

		public ColorHistogram (int nbins, FrameManager fr) {
			BinsNumber = nbins;
			dim = BinsNumber * BinsNumber * BinsNumber;

			data_normalized = new Vector2[dim];
			data_unnormalized = new Vector2[dim];

			posterior = new float[dim];

			frame = fr;
		}

		// Update histogram based on an existing histogram
		public void UpdateHistogram( ColorHistogram newHist, float rf, float rb ) {
			for (int i = 0; i < dim; i++) {
				this.data_normalized [i].x = this.data_normalized [i].x * (1 - rf) + newHist.data_normalized [i].x * rf;
				this.data_normalized[i].y = this.data_normalized[i].y * (1 - rb) + newHist.data_normalized[i].y * rb;
				this.posterior[i] = this.data_normalized[i].x / (this.data_normalized[i].x + this.data_normalized[i].y);
			}
		}

		public void BuildHistogram() {
			int idx_mask;
			int ru, gu, bu;
			int pidx;

			float sumForeground = 0;
			float sumBackground = 0;


			for (int j = 0; j < frame.Height; j++) {
				for (int i = 0; i < frame.Width; i++) {
					int idx = i + j * frame.Width;
					Color pixel = frame.GetPixelValue(frame.ColorPoints[idx]);
					ru = pixel.r / BinsNumber;
					gu = pixel.g / BinsNumber;
					bu = pixel.b / BinsNumber;
					pidx = ru * BinsNumber * BinsNumber + gu * BinsNumber + bu;

					// TODO these colors can be replaced by macros?
					switch (frame.Mask[idx].r) {

					case Color.white: // foreground is white
						data_normalized [pidx].x++;
						sumForeground++;
						break;

					case Color.black: // far background is black
						break;

						default: // other colors are immediate background
						data_normalized [pidx].y++;
						sumBackground++;
						break;
					}
				}
			}

			sumForeground = (sumForeground != 0) ? 1.0f/sumForeground : 0;
			sumBackground = (sumBackground != 0) ? 1.0f/sumBackground : 0;

			for ( int i=0; i<dim; i++) {
				data_normalized[i].x = data_normalized[i].x + sumForeground + 0.0001f;
				data_normalized[i].y = data_normalized[i].y + sumBackground + 0.0001f;
				posterior[i] = data_normalized[i].x / (data_normalized[i].x + data_normalized[i].y);
			}
		}

		public void BuildHistogramFromLabeledRGBD() {
			int idx_mask;
			int ru, gu, bu;
			int pidx;

			float sumForeground = 0;
			float sumBackground = 0;


			for (int j = 0; j < frame.Height; j++) {
				for (int i = 0; i < frame.Width; i++) {
					int idx = i + j * frame.Width;

					Color pixel = frame.GetPixelValue(frame.ColorPoints[idx]);
					float w = frame.PfVec [idx];

					if (w >= defines.HIST_USELESS_PIXEL)
						continue;
					
					ru = pixel.r / BinsNumber;
					gu = pixel.g / BinsNumber;
					bu = pixel.b / BinsNumber;
					pidx = ru * BinsNumber * BinsNumber + gu * BinsNumber + bu;

					// TODO these colors can be replaced by macros?
					switch ( w ) {

					case defines.HIST_FG_PIXEL: // foreground pixel
						data_normalized [pidx].x++;
						sumForeground++;
						break;

					case defines.HIST_BG_PIXEL: // background pixel
						data_normalized [pidx].y++;
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
				data_normalized[i].x = data_normalized[i].x + sumForeground + 0.0001f;
				data_normalized[i].y = data_normalized[i].y + sumBackground + 0.0001f;
				posterior[i] = data_normalized[i].x / (data_normalized[i].x + data_normalized[i].y);
			}
		}

		public void UpdateHistogramFromLabeledRGBD(float rf, float rb)
		{
			ColorHistogram tmpHist = new ColorHistogram(this.BinsNumber, this.frame);
			this.BuildHistogramFromLabeledRGBD ();
			this.UpdateHistogram (tmpHist, rf, rb);
		}

		// TODO do we need bb?
		public void buildHistogramFromLabeledRGBD()
		{
		}
		public void updateHistogramFromLabeledRGBD(float rf, float rb)
		{
//			ISRHistogram* tmphist = new ISRHistogram(this->noBins);
//			buildHistogramFromLabeledRGBD(inimg,bb);
//			updateHistogram(tmphist, rf, rb);
		}


	}

}
