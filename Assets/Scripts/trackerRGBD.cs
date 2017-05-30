using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using HoloProxies.Objects;

namespace HoloProxies.Engine
{
    /// <summary>
    /// The trackerRGBD class implements all the algorithm of computing the energy function and minimizing it using
    /// Levenberg-Marquardt.
    /// File from: ISRRGBDTracker.cpp/.h, ISRTracker.h, ISRRGBDTracker_CPU.cpp/.h, ISRRGBDTracker_shared.h
    /// TODO unfinished
    /// </summary>
    public class trackerRGBD {

        // the current accepted tracker's state
        // incremental change of poses will always
        // be applied to this state
        private trackerState acceptedState;

        // temp tracker's state after applying the incremental pose change
        // energy function is always evaluated on this state
        private trackerState tempState;

        // hold the set of shapes
        shapeSDF shape;

        // hold the current frame


        public trackerRGBD( int nObjs )
        {

        }
	    
    }
}


