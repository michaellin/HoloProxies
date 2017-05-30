using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace HoloProxies.Objects
{
    /// <summary>
    /// The histogramRGB class is data structure that hold the histogram data of the RGB values.
    /// This is used to calculate the probability function used for the w value of the point cloud.
    /// Not sure how using the histogram RGB as point cloud w variable works...
    /// File from: ISRHistogram.h
    /// </summary>
    public class histogramRGB
    {
        public Vector2[] data_notnormalized;
        public Vector2[] data_normalized;

        public float[] posterior;

        public bool initialized;

        public int bin_num, dim;

        /*** public functions ***/
        public histogramRGB( int nBins )
        {
            bin_num = nBins;
            dim = nBins * nBins * nBins;

            data_normalized = new Vector2[dim];
            data_notnormalized = new Vector2[dim];
            posterior = new float[dim];

            this.clear();
        }

        public float[] getPosteriorHistogram()
        {
            return posterior;
        }

        /// <summary>
        /// Function to update current HistogramRGB given a new HistogramRGB. However, update it
        /// at a decaying rate. Update the foreground at rf rate and background at rb rate.
        /// TODO
        /// </summary>
        public void updateHistogram( histogramRGB newHist, float rf, float rb )
        {
            
        }

        /// <summary>
        /// Function to build a histogram from color and mask. These two are RGBA32 format.
        /// TODO
        /// </summary>
        public void buildHistogram( Texture2D color, Texture2D mask )
        {

        }

        /// <summary>
        /// Function to build a histogram from RGBD. Input format is RGBAFloat.
        /// TODO
        /// </summary>
        public void buildHistogramFromLabelledRGBD( Texture2D inimg )
        {

        }

        /// <summary>
        /// Function to build a histogram from RGBD faster since it only does it for within the bounding box.
        /// Input format is RGBAFloat.
        /// TODO
        /// </summary>
        public void buildHistogramFromLabelledRGBD( Texture2D inimg, Vector4 bb )
        {

        }

        /// <summary>
        /// Function to update a histogram from RGBD at rates rf and rb. This can call buildHistogramFromLabelledRGBD
        /// and updateHistogram. Input format is RGBAFloat.
        /// TODO
        /// </summary>
        public void updateHistogramFromLabelledRGBD( Texture2D inimg, float rf, float rb )
        {

        }

        /// <summary>
        /// Function to update a histogram from RGBD at rates rf and rb. It uses bounding box. This can call 
        /// buildHistogramFromLabelledRGBD and updateHistogram to update the histogram data. Input format is RGBAFloat.
        /// TODO
        /// </summary>
        public void updateHistogramFromLabelledRGBD( Texture2D inimg, float rf, float rb, Vector4 bb )
        {

        }

        public void clearNormalised()
        {
            for (int i = 0; i < dim; i++)
            {
                data_normalized[i] = new Vector2();
            }
        }

        public void clearNotNormalised()
        {
            for (int i = 0; i < dim; i++)
            {
                data_notnormalized[i] = new Vector2();
            }
        }

        public void clearPosterior()
        {
            for( int i = 0; i < dim; i++)
            {
                posterior[i] = 0.0f;
            }
        }

        public void clear()
        {
            clearNormalised();
            clearNotNormalised();
            clearPosterior();
            initialized = false;
        }


    }
}
