using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using HoloProxies.Objects;
using HoloProxies.Utils;

namespace HoloProxies.Engine
{
    /// <summary>
    /// The coreEngine class implements the processFrame function. The main function that grabs RGBD data to track proxy objects.
    /// File from: ISRCoreEngine.cpp/.h
    /// </summary>
    public class coreEngine
    {
        public trackerState trackingState;

        private int histogramNBins;
        private int numTrackingObjs;
        private frameRGBD frame;

        public coreEngine( int[] settings, Vector2i d_size, Vector2i rgb_size )
        {
            histogramNBins = settings[0];
            numTrackingObjs = settings[1];


        }

    }
}
