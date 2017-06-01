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
        private shapeSDF shape;
        private frameRGBD frame;
        private trackerRGBD tracker;

        public coreEngine( int[] settings, Vector2i d_size, Vector2i rgb_size, int vol_size )
        {
            histogramNBins = settings[0];
            numTrackingObjs = settings[1];

            frame = new frameRGBD( rgb_size, d_size );
            frame.hist = new histogramRGB( histogramNBins );
            shape = new shapeSDF( vol_size );
            tracker = new trackerRGBD( numTrackingObjs );

        }

        /// <summary>
        /// Main function that gets a frame and updates the tracker based on new data.
        /// 
        /// </summary>
        public void processFrame()
        {
            // Create a timer to measure fps
            // TODO

            frame.boundingbox = lowLevelEngine.findBoundingBoxFromCurrentState();
            //frame.intrinsic = intrinsics_d; // TODO find out if this is needed

            lowLevelEngine.prepareAlignedRGBDData( out frameRGBD, rawDepth, frameRGBD, homo_depth_to_color ); // TODO Might not need this function at all

            // Size the image size correctly. Downsample if needed.
            trackingState.boundingBox = frame.boundingbox;

			// TODO change this to ISR frame manager
            //lowLevelEngine.preparePointCloudFromAlignedRGBDImage( frame.ptCloud, frame.rgbd, frame.hist, frame.intrinsic, frame.boundingbox );

            if (needStarTracker)
            {
                tracker.TrackObjects( frame, shape, trackingState );
            }


        }

    }
}
