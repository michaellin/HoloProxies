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
    /// File from: ISRTrackingState.h
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
			for (int i = 0, j = 0; i < nPose; i++, j += 6) {
				poses [i].applyIncrementalChangeToInvH (step [j]);
			}
        }

        /// <summary>
        /// Function to apply incremental pose changes to all poses
        /// being tracked.
        /// TOOD
        /// </summary>
        public void applyIncrementalPoseChangesToH( float[] step )
        {
			for (int i = 0, j = 0; i < nPose; i++, j += 6) {
				poses [i].applyIncrementalChangeToH (step [j]);
			}
        }

        /// <summary>
        /// Function to set this trackerState from another trackerState.
        /// TOOD
        /// </summary>
        public void setFrom( trackerState inposes )
        {
			int count = inposes.numPoses();
			boundingBox = inposes.boundingBox;
			for (int i = 0; i < count; i++)
			{
				this.getPose(i).setFromH(inposes.getPose(i).getH());
			}
        }

        /// <summary>
        /// Function to set H of each pose from the given param.
        /// TOOD
        /// </summary>
        public void setHFromParam( float[] param, int id )
        {
			poses[id].setHFromParam(param);
        }

        /// <summary>
        /// Function to set invH of each pose from the given param.
        /// TOOD
        /// </summary>
        public void setInvHFromParam( float[] param, int id )
        {
			poses[id].setInvHFromParam(param);
        }

    }
}
