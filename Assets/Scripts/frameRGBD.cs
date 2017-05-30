using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using HoloProxies.Utils;

namespace HoloProxies.Objects
{
    /// <summary>
    /// The frameRGBD class carries the intermediate data for a 
    /// RGBD frame, including occlusion map and pointCloud.
    /// TODO figure out if any of the TODOs here need to be addressed.
    /// File from: ISRFrame.h
    /// </summary>
    public class frameRGBD
    {
        public int height;
        public int width;

        Vector2i depth_size;
        Vector2i rgb_size;

        //TODO ISRView* view; Do we need this?

        histogramRGB hist;
        Texture2D ptCloud; // 2D point cloud of format RGBAFloat

        //TODO ISRImageHierarchy needed?
        //TODO image levels needed?

        //TODO VisualizationState needed?

        //TODO need calib to be its own file?

        public frameRGBD( Vector2i color_size, Vector2i d_size )
        {
            depth_size = d_size;
            rgb_size = color_size;

            ptCloud = new Texture2D( d_size.x, d_size.y, TextureFormat.RGBAFloat, false );

        }


    }
}
