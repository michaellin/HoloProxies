using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using HoloProxies.Objects;

namespace HoloProxies.Engine
{
    /// <summary>
    /// The trackerState class contains the current state
    /// of the multi-object object tracker.
    /// Energy function is evaluated given this class.
    /// </summary>
    public class trackerState
    {
        private int nPose;
        private objectPose[] poses;

        public float energy;
        public Vector4 boundingBox;

        /// <summary>
        /// Constructor initializes an array of poses.
        /// </summary>
        public trackerState( int num )
        {
            energy = 0;
            nPose = num;
            poses = new objectPose[num];
        }

        public objectPose[] getPoseList()
        {
            return poses;
        }

        public objectPose getPose( int id )
        {
            return poses[id];
        }

        public int numPoses()
        {
            return nPose;
        }

        /// <summary>
        /// Function to apply incremental pose changes to all poses
        /// being tracked.
        /// TOOD
        /// </summary>
        public void applyIncrementalPoseChangesToInvH( float[] step )
        {

        }

        /// <summary>
        /// Function to apply incremental pose changes to all poses
        /// being tracked.
        /// TOOD
        /// </summary>
        public void applyIncrementalPoseChangesToH( float[] step )
        {

        }

        /// <summary>
        /// Function to set this trackerState from another trackerState.
        /// TOOD
        /// </summary>
        public void setFrom( trackerState inposes )
        {

        }

        /// <summary>
        /// Function to set H of each pose from the given param.
        /// TOOD
        /// </summary>
        public void setHFromParam( float[] param, int id )
        {

        }

        /// <summary>
        /// Function to set invH of each pose from the given param.
        /// TOOD
        /// </summary>
        public void setInvHFromParam( float[] param, int id )
        {

        }

    }
}
