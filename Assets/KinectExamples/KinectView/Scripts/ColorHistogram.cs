﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using HoloProxies.Utils;

namespace HoloProxies.Objects
{
	
	public class ColorHistogram  {

		public Vector2[] data_unnormalized;
		public Vector2[] data_normalized;

		public float[] posterior;

		public int noBins, dim;

		public ColorHistogram (int nbins) {
		}

		public void UpdateHistogram( ColorHistogram newHist, float rf, float rb ) {
		}

		public void BuildHistogram() {
		}

		public void BuildHistogramFromLabeledRGBD() {
		}

		public void updateHistogramFromLabeledRGBD(float rf, float rb)
		{
//			ISRHistogram* tmphist = new ISRHistogram(this->noBins);
//			buildHistogramFromLabeledRGBD(inimg);
//			updateHistogram(tmphist, rf, rb);
		}

		public void updateHistogramFromLabeledRGBD(float rf, float rb, Vector4 bb)
		{
//			ISRHistogram* tmphist = new ISRHistogram(this->noBins);
//			buildHistogramFromLabeledRGBD(inimg,bb);
//			updateHistogram(tmphist, rf, rb);
		}


	}

}