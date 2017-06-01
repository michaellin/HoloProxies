﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using HoloProxies.Utils;

namespace HoloProxies.Objects {
    /// <summary>
    /// The shapeSDF class is the container that defines the shape to be tracked as a signed distance function.
    /// Instances should contain a buffer of floats of size cubeEdgeSize^3 and each voxel in this cube contains 
    /// the distance to the nearest object surface.
    /// TODO: Need to find a consistent way to load a shape into this SDF container.
    /// File from: ISRShapeUnion.h and ISRShape.h
    /// </summary>
    public class shapeSDF
    {
        // 3-Dimensional array of floats that contains the SDF values.
        public float[] dt;       // Allow others to directly access this variale. Should be read only
        public Vector3 volSize;

        /// <summary>
        /// Class constructor should initialize the shape_buffer. 
        /// </summary>
        public shapeSDF()
        {
            volSize = new Vector3( defines.DT_VOL_SIZE, defines.DT_VOL_SIZE, defines.DT_VOL_SIZE );
            // TODO define a shape to start with. Make a sphere for now.
        }

        /// <summary>
        /// Function to access the SDF values. It will be called per pixel to calculate
        /// energy so it should be efficient.
        /// </summary>
        public float[] getSDFVoxels()
        {
            return dt;
        }

    }
}

