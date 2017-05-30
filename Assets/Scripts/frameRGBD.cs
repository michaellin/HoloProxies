using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace HoloProxies.Objects
{
    /// <summary>
    /// The frameRGBD class carries the intermediate data for a 
    /// RGBD frame, including occlusion map and pointCloud.
    /// </summary>
    public class frameRGBD
    {
        public int height;
        public int width;

        Vector2 depth_size;
        Vector2 rgb_size;

        //TODO ISRView* view; Do we need this?

        HistogramRGB hist;
        Texture2D ptCloud; // Originally it was a Float4Image

        //TODO ISRImageHierarchy needed?
        //TODO image levels needed?

        //TODO VisualizationState needed?

        //TODO need calib to be its own file?

        public frameRGBD( calib, Vector2 color_size, Vector2 d_size )
        {
            depth_size = d_size;
            rgb_size = color_size;

            ptCloud = new Texture2D( d_size );
        }


    }
}
