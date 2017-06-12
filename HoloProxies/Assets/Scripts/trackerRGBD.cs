using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;
using UnityEngine;

using Windows.Kinect;

using HoloProxies.Engine;
using HoloProxies.Utils;
using HoloProxies.Objects;
using Helpers;

namespace HoloProxies.Engine
{
    /// <summary>
    /// The trackerRGBD class implements all the algorithm of computing the energy function and minimizing it using
    /// Levenberg-Marquardt.
    /// File from: ISRRGBDTracker.cpp/.h, ISRTracker.h, ISRRGBDTracker_CPU.cpp/.h, ISRRGBDTracker_shared.h
    /// </summary>
    public class trackerRGBD
    {

        // the current accepted tracker's state
        // incremental change of poses will always
        // be applied to this state
        // this is accessible by others.
        public trackerState trackingState;

        // temp tracker's state after applying the incremental pose change
        // energy function is always evaluated on this state
        private trackerState tempState;

        // hold the set of shapes
        shapeSDF[] shapes;

        // hold the current frame this is just for convenience so it does 
        // not need to be passed in as function param many times
        FrameManager frame;

        // number of objects
        int nObjects;

        /*** Variables used by TrackObject preallocated to improve speed ***/
        // size of the gradient
        int ATb_size; // (6*nObjects)

        // size of the Hessian
        int ATA_size; // (Atb_size^2)

        // Hessian approximated with JTJ
        float[] ATA;

        // gradient
        float[] ATb;

        /*** Variables used by TrackObject preallocated to improve speed ***/
        float[] tmpATA;


        /*** Variables used by computePerPixelJacobian preallocate for speed ***/
        float[] voxelBlocks = new float[defines.DT_VOL_3DSIZE];

        float[] minVoxelBlocks = new float[defines.DT_VOL_3DSIZE];


        #region ISRRGBtracker.cpp
        public trackerRGBD( int nObjs, string[] fileNames )
        {
            nObjects = nObjs;

            shapes = new shapeSDF[nObjects];
            for (int i = 0; i < nObjects; i++)
            {
                shapes[i] = new shapeSDF( fileNames[i] );
            }

            ATb_size = nObjs * 6;
            ATA_size = ATb_size * ATb_size;

            ATb = new float[ATb_size];
            ATA = new float[ATA_size];

            tmpATA = new float[ATb_size * ATb_size];

            trackingState = new trackerState( nObjs );
            tempState = new trackerState( nObjs );
        }

        void computeSingleStep( float[] step, float[] ATA, float[] ATb, float lambda, int dim )
        {
            int dim2 = dim * dim;
            //float[] tmpATA = new float[dim2]; TODO I preallocated at the top
            Array.Copy( ATA, tmpATA, dim2 );

            for (int i = 0; i < dim2; i += (dim + 1))
            {
                float ele = tmpATA[i];

                if (!(Math.Abs( ele ) < 1e-15f))
                {
                    ele *= (1.0f + lambda);
                }
                else
                {
                    ele = lambda * 1e-10f;
                }
                tmpATA[i] = ele;
            }


            Cholesky cholA = new Cholesky( tmpATA, dim );
            cholA.Backsub( step, ATb );
        }

        void fastReinitialize( float oldenergy )
        {
            //			float e = oldenergy, tmpe = 0;
            //			Vector3f bigestR(0.0f);
            //
            //			int R_SEP = 6;
            //			float R_DEGREE = 360.0f / R_SEP;
            //
            //			for (int i = 1; i < R_SEP; i++)
            //			{
            //				for (int j = 0; j < 3; j++)
            //				{
            //					Vector3f tmpV(0.0f);
            //					tmpV.v[j] = i * R_DEGREE;
            //					this->tempState->setFrom(*this->accpetedState);
            //					this->tempState->getPose(0)->applyIncrementalRotationToInvHInDegree(tmpV);
            //					this->tempState->updatePoseDeviceFromHost();
            //					evaluateEnergy(&tmpe, this->tempState);
            //
            //					if (tmpe > e)
            //					{
            //						e = tmpe;
            //						bigestR = tmpV;
            //					}
            //				}
            //
            //			}
            //
            //			this->tempState->setFrom(*this->accpetedState);
            //
            //			if (e-oldenergy>=0.05)
            //			{
            //				printf("Reinitialized, energy increase: %f \t rotation: (%d,%d,%d)\n", e - oldenergy, (int)bigestR.x, (int)bigestR.y, (int)bigestR.z);
            //				this->tempState->getPose(0)->applyIncrementalRotationToInvHInDegree(bigestR);
            //				this->tempState->updatePoseDeviceFromHost();
            //				this->accpetedState->setFrom(*this->tempState);
            //				oldenergy = e;
            //			}
        }

        public void TrackObjects( FrameManager frame, bool updateappearance )
        {
            // originally here was:
            //this->shapeUnion = shapeUnion;

            this.frame = frame; // For convenience

            tempState.setFrom( trackingState );

            float[] cache = new float[ATb_size];
            
            float lastenergy = 0;
            float currentenergy = 0;

            bool converged = false;
            float lambda = 10000.0f;

            evaluateEnergy( out lastenergy, trackingState );
            //Debug.Log( "last energy " + lastenergy.ToString( "F6" ) );
            if (lastenergy < 0.1f) { trackingState.energy = 0; return; }

            /*** Levenberg-Marquardt ***/

            const int MAX_STEPS = 100;
            const float MIN_STEP = 0.00005f;
            const float MIN_DECREASE = 0.0001f;
            const float TR_REGION_INCREASE = 0.10f;
            const float TR_REGION_DECREASE = 10.0f;

            { // create a local scope to control these variables
                for (int iter = 0; iter < MAX_STEPS; iter++)
                {
                    computeJacobianAndHessian( ATb, ATA, tempState );

                    while (true)
                    {
                        computeSingleStep( cache, ATA, ATb, lambda, ATb_size );

                        // check if we have converged
                        float MAXnorm = 0.0f;
                        for (int i = 0; i < ATb_size; i++)
                        {
                            float tmp = Math.Abs( cache[i] );// > 0? cache[i] : -cache[i]; // absolute value
                            if (tmp > MAXnorm)
                            {
                                MAXnorm = tmp;
                            }
                        }
                        if (MAXnorm < MIN_STEP)
                        {
                            converged = true;
                            break;
                        }

                        tempState.applyIncrementalPoseChangesToInvH( cache );

                        evaluateEnergy( out currentenergy, tempState );

                        if (currentenergy > lastenergy)
                        {
                            if (Math.Abs( currentenergy - lastenergy ) / Math.Abs( lastenergy + 0.01f ) < MIN_DECREASE)
                            {
                                converged = true;
                            }
                            lastenergy = currentenergy;
                            lambda *= TR_REGION_INCREASE;
                            trackingState.setFrom( tempState ); // accept the converged state
                        }
                        else
                        {
                            lambda *= TR_REGION_DECREASE;
                            tempState.setFrom( trackingState );
                        }
                    }
                    if (converged) { break; }
                }
            }

            // after convergence, the pf of the pointcloud is recycled for histogram update
            if ( lastenergy >= 0.5f ) //&& updateappearance) //0.5f && updateappearance)
            {
                //Debug.Log( "recycle!!!!!!" );
                labelMaskForegroundPixels( trackingState );

                frame.histogram.UpdateHistogramFromLabeledMask( 0.5f, 0.1f, frame.ColorTexture, frame.Mask );
            }

            trackingState.energy = lastenergy;

        }
        #endregion

        public bool TestEvaluateEnergy( string ptcloudFile )
        {
            //int npoints = width * height;
            CameraSpacePoint[] pointCloud = new CameraSpacePoint[13568];
            float[] pfvec = new float[13568];
            // Read the file and parse data into point cloud
            using (BinaryReader file = new BinaryReader( File.Open( ptcloudFile, FileMode.Open ) ))
            {
                for (int k = 0; k < 13568; k++)
                {
                    pointCloud[k] = new CameraSpacePoint();
                    pointCloud[k].X = file.ReadSingle();
                    pointCloud[k].Y = file.ReadSingle();
                    pointCloud[k].Z = file.ReadSingle();
                    pfvec[k] = file.ReadSingle();
                }
            }

            int count = 13568;

            CameraSpacePoint[] ptcloud_ptr = pointCloud;
            float[] pfArray = pfvec;

            Matrix4x4 pose = new Matrix4x4();
            pose.m00 = 0.760526f; pose.m10 = 0.523887f; pose.m20 = 0.38359f; pose.m30 = 0.0f;
            pose.m01 = 0.631573f; pose.m11 = -0.459739f; pose.m21 = -0.624303f; pose.m31 = 0.0f;
            pose.m02 = -0.150713f; pose.m12 = 0.717064f; pose.m22 = -0.680517f; pose.m32 = 0.0f;
            pose.m03 = 0.007287f; pose.m13 = -0.011503f; pose.m23 = 0.583796f; pose.m33 = 1.0f;
            trackerState state = new trackerState( 1 );
            state.getPose( 0 ).setFromH( pose );
            objectPose[] poses = state.getPoseList();
            int objCount = state.numPoses();

            float e = 0, es = 0;
            int totalpix = 0;
            int totalpfpix = 0;

            for (int i = 0; i < count; i++)
            {
                es = computePerPixelEnergy( ptcloud_ptr[i], pfArray[i], poses, objCount );
                if (es > 0)
                {
                    e += es; totalpix++;
                    if (pfArray[i] > 0.5) totalpfpix++;
                }
            }

            float energy = totalpfpix > 100 ? e / totalpix : 0.0f;

            //Debug.Log( "energy is " + energy.ToString( "F6" ) );
            return true;
        }


        #region main functions
        /// <summary>
        /// evaluates the total energy of a given frame
        /// </summary>
        /// <param name="energy"></param>
        /// <param name="state"></param>
        private void evaluateEnergy( out float energy, trackerState state )
        {
            int count = frame.Width * frame.Height; //frame.Camera3DPoints.Length; TODO Michael changed this

            CameraSpacePoint[] ptcloud_ptr = frame.Camera3DPoints;

            objectPose[] poses = state.getPoseList();
            int objCount = state.numPoses();

            float e = 0, es = 0;
            int totalpix = 0;
            int totalpfpix = 0;

            //int testcount = 0;
            //int testcount2 = 0;

            for (int i = 0; i < count; i++)
            {
                es = computePerPixelEnergy( ptcloud_ptr[i], frame.PfVec[i], poses, objCount );
                if (es > 0)
                {
                    e += es; totalpix++;
                    if (frame.PfVec[i] > 0.5) { totalpfpix++; }
                }
            }
            energy = totalpfpix > 100 ? e / totalpix : 0.0f;
        }

        /// <summary>
        ///  computePerPixelEnergy finds the energy of a single pixel given a pose and the SDF value of the corresponding voxel
        /// </summary>
        /// <param name="inpt"></param>
        /// <param name="pf"></param>
        /// <param name="pose"></param>
        /// <param name="numObj"></param>
        /// <returns> float enegy value of the input pixel </returns>
        private float computePerPixelEnergy( CameraSpacePoint inpt, float pf, objectPose[] poses, int numObj )
        {
            if (pf > 0)
            {
                float dt = defines.MAX_SDF;
                float partdt = defines.MAX_SDF;
                int idx;
                float[] voxelBlocks;

                //int testcount = 0;
                //int testcount2 = 0;

                for (int i = 0; i < numObj; i++)
                {

                    Vector3 objpt = poses[i].getInvH().MultiplyPoint( new Vector3( inpt.X, inpt.Y, inpt.Z ) );

                    idx = pt2IntIdx( objpt );
                    if (idx >= 0)
                    {
                        voxelBlocks = shapes[i].getSDFVoxels();
                        partdt = voxelBlocks[idx];
                        dt = partdt < dt ? partdt : dt; // now use a hard min to approximate
                    }
                }

                if (dt == defines.MAX_SDF) { return -1.0f; };
                double exp_dt = Math.Exp( -dt * defines.DTUNE );
                double deto = exp_dt + 1.0f;
                double sheaviside = 1.0f / deto;
                double sdelta = 4.0f * exp_dt * sheaviside * sheaviside;
                float e = (float)(pf * sdelta * defines.TMP_WEIGHT + (1 - pf) * sheaviside * (2 - defines.TMP_WEIGHT));

                return e;
            }
            else { return 0.0f; }
        }

        #endregion

        #region helper functions
        /// <summary>
        /// computeJacobianAndHessian helper function to get jacobian and hessian used for the next step. This function should not change
        /// anything in the tracker object.
        /// </summary>
        /// <param name="gradient"></param>
        /// <param name="hessian"></param>
        /// <param name="tracker"></param>
        private void computeJacobianAndHessian( float[] gradient, float[] hessian, trackerState tracker )
        {

            int count = frame.Camera3DPoints.Length; //frame.Width * frame.Height; //
            CameraSpacePoint[] ptcloud = frame.Camera3DPoints; // 3D voxels in meters
            float[] pfArray = frame.PfVec; // pf of each voxel
            objectPose[] poses = tracker.getPoseList();
            int objCount = tracker.numPoses();

            int paramNum = objCount * 6;
            int paramNumSq = paramNum * paramNum;

            float[] globalGradient = new float[paramNum];
            float[] globalHessian = new float[paramNumSq];
            float[] jacobian = new float[paramNum];

            Array.Clear( globalGradient, 0, paramNum );
            Array.Clear( globalHessian, 0, paramNumSq );

            for (int i = 0; i < count; i++)
            {
                if (pfArray[i] != defines.OUTSIDE_BB)
                {

                    if (computePerPixelJacobian( out jacobian, ptcloud[i], pfArray[i], poses, objCount ))
                    {
                        for (int a = 0, counter = 0; a < paramNum; a++)
                        {
                            globalGradient[a] += jacobian[a];
                            for (int b = 0; b <= a; b++, counter++) { globalHessian[counter] += jacobian[a] * jacobian[b]; }
                        }
                    }
                }
            }


            Array.Copy( globalGradient, gradient, paramNum );
            for (int r = 0, counter = 0; r < paramNum; r++)
            {
                for (int c = 0; c <= r; c++, counter++)
                {
                    hessian[r + c * paramNum] = globalHessian[counter];
                }
            }
            for (int r = 0; r < paramNum; ++r)
            {
                for (int c = r + 1; c < paramNum; c++)
                {
                    hessian[r + c * paramNum] = hessian[c + r * paramNum];
                }
            }
        }

        /// <summary>
        /// Helper function to compute the jacobian. ptCloud is in camera reference frame and needs to be converted using the pose invH to object coordinates.
        /// pfVec is the probability of each voxel (matches in index). input now is in camera coordinates, it need to be transformed by pose invH to object coordinates
        /// input is also been properly scaled to math the voxel resolution.
        /// </summary>
        /// <param name="jacobian"></param>
        /// <param name="ptcloud"></param>
        /// <param name="pfVec"></param>
        /// <param name="shapes"></param>
        /// <param name="poses"></param>
        /// <param name="numObj"></param>
        /// <returns></returns>
        private bool computePerPixelJacobian( out float[] jacobian, CameraSpacePoint ptcloud, float pfVal, objectPose[] poses, int numObj )
        {
            jacobian = new float[numObj * 6];
            if (pfVal < 0) { return false; }
            float dt = defines.MAX_SDF; float partdt = defines.MAX_SDF;
            int idx; int minidx = 0;
            //float[] voxelBlocks = new float[defines.DT_VOL_3DSIZE];
            //float[] minVoxelBlocks = new float[defines.DT_VOL_3DSIZE];
            Vector3 pt = new Vector3( ptcloud.X, ptcloud.Y, ptcloud.Z );
            Vector3 minpt = new Vector3();
            Vector3 ddt;
            bool minfound = false;
            bool ddtfound = false;

            for (int i = 0; i < numObj; i++)
            {
                Vector3 objpt = poses[i].getInvH().MultiplyPoint( pt );
                idx = pt2IntIdx( objpt );

                if (idx >= 0)
                {
                    voxelBlocks = shapes[i].getSDFVoxels();
                    partdt = voxelBlocks[idx];
                    if (partdt < dt)
                    {
                        minidx = i;
                        minpt = objpt;
                        minfound = true;
                        minVoxelBlocks = voxelBlocks;

                        dt = partdt;
                    }
                }
            }
            if (!minfound) { return false; }

            ddt = getSDFNormal( minpt, minVoxelBlocks, out ddtfound );

            if (!ddtfound) { return false; }

            double exp_dt = Math.Exp( -dt * defines.DTUNE );
            double deto = exp_dt + 1.0f;
            double dbase = exp_dt / (deto * deto);

            double d_heaviside_dt = dbase * defines.DTUNE;
            double d_delta_dt = 8.0f * defines.DTUNE * Math.Exp( -2.0f * defines.DTUNE * dt ) / (deto * deto * deto) - 4.0f * defines.DTUNE * dbase;

            float prefix = (float)(pfVal * d_delta_dt * defines.TMP_WEIGHT + (1.0f - pfVal) * d_heaviside_dt * (2.0f - defines.TMP_WEIGHT));

            ddt *= prefix;

            Array.Clear( jacobian, 0, numObj * 6 );
            int idxoffset = minidx * 6;

            jacobian[idxoffset + 0] = ddt.x;
            jacobian[idxoffset + 1] = ddt.y;
            jacobian[idxoffset + 2] = ddt.z;
            jacobian[idxoffset + 3] = 4.0f * (ddt.z * minpt.y - ddt.y * minpt.z);
            jacobian[idxoffset + 4] = 4.0f * (ddt.x * minpt.z - ddt.z * minpt.x);
            jacobian[idxoffset + 5] = 4.0f * (ddt.y * minpt.x - ddt.x * minpt.y);

            return true;
        }


        /*** These helper functions come from ISRVoxelAccess_shared.h ***/
        private int pt2IntIdx( Vector3 pt )
        {
            float vol_scale = defines.VOL_SCALE;
            float dt_vol_size = defines.DT_VOL_SIZE;
            int x = (int)(pt.x * vol_scale + dt_vol_size / 2.0f - 1.0f);
            int y = (int)(pt.y * vol_scale + dt_vol_size / 2.0f - 1.0f);
            int z = (int)(pt.z * vol_scale + dt_vol_size / 2.0f - 1.0f);

            if (x > 0 && x < defines.DT_VOL_SIZE - 1 &&
                y > 0 && y < defines.DT_VOL_SIZE - 1 &&
                z > 0 && z < defines.DT_VOL_SIZE - 1)
                return (z * defines.DT_VOL_SIZE + y) * defines.DT_VOL_SIZE + x;
            else
                return -1;
        }

        private int pt2IntIdx_offset( Vector3 pt, Vector3 offpt )
        {
            float vol_scale = defines.VOL_SCALE;
            float dt_vol_size = defines.DT_VOL_SIZE;
            int x = (int)(pt.x * vol_scale + dt_vol_size / 2.0f - 1.0f + offpt.x);
            int y = (int)(pt.y * vol_scale + dt_vol_size / 2.0f - 1.0f + offpt.y);
            int z = (int)(pt.z * vol_scale + dt_vol_size / 2.0f - 1.0f + offpt.z);

            if (x > 0 && x < dt_vol_size - 1.0f &&
                y > 0 && y < dt_vol_size - 1.0f &&
                z > 0 && z < dt_vol_size - 1.0f)
                return (int)((z * dt_vol_size + y) * dt_vol_size + x);
            else
                return -1;
        }

        private Vector3 getSDFNormal( Vector3 pt_f, float[] voxelBlock, out bool ddtFound )
        {
            Vector3 ddt = new Vector3();

            float dt1; float dt2;
            int idx;

            idx = pt2IntIdx_offset( pt_f, new Vector3( 1, 0, 0 ) );
            if (idx == -1) { ddtFound = false; return new Vector3(); }
            dt1 = voxelBlock[idx];
            idx = pt2IntIdx_offset( pt_f, new Vector3( -1, 0, 0 ) );
            if (idx == -1) { ddtFound = false; return new Vector3(); }
            dt2 = voxelBlock[idx];
            ddt.x = (dt1 - dt2) * 0.5f;

            idx = pt2IntIdx_offset( pt_f, new Vector3( 0, 1, 0 ) );
            if (idx == -1) { ddtFound = false; return new Vector3(); }
            dt1 = voxelBlock[idx];
            idx = pt2IntIdx_offset( pt_f, new Vector3( 0, -1, 0 ) );
            if (idx == -1) { ddtFound = false; return new Vector3(); }
            dt2 = voxelBlock[idx];
            ddt.x = (dt1 - dt2) * 0.5f;

            idx = pt2IntIdx_offset( pt_f, new Vector3( 0, 0, 1 ) );
            if (idx == -1) { ddtFound = false; return new Vector3(); }
            dt1 = voxelBlock[idx];
            idx = pt2IntIdx_offset( pt_f, new Vector3( 0, 0, -1 ) );
            if (idx == -1) { ddtFound = false; return new Vector3(); }
            dt2 = voxelBlock[idx];
            ddt.x = (dt1 - dt2) * 0.5f;

            ddtFound = true; return ddt;
        }

        #endregion


        /// <summary>
        /// lableForegroundPixels to label pixels as forground or background for better processing :)
		/// This labeling is done on the frame Mask
        /// </summary>
        /// <param name="state"></param>
        void labelMaskForegroundPixels( trackerState state )
        {
            int count = frame.Width * frame.Height;
            float dt;

            //for (int i = 0; i < count; i++)
            //{
            for ( int i = 0; i < frame.Height; i++ )
            {
                for (int j = 0; j < frame.Width; j++)
                {
                    int idx = i * frame.Width + j;
                    // Keep track of useless pixels on the mask and check if we're inside the bb
                    if ((frame.PfVec[i] > 0)) //&& (frame.PfVec[i] != defines.OUTSIDE_BB)) //TODO-alexa
                    {
                        dt = findPerPixelDT( frame.Camera3DPoints[idx], frame.PfVec[idx], state.getPoseList(), defines.NUM_OBJ );
                        if (Mathf.Abs( dt ) <= 5)
                        {
                            frame.Mask[i] = defines.HIST_FG_PIXEL;
                            Color pixel = frame.ColorTexture.GetPixel( j, i );
                            frame.ColorTextureVisual.SetPixel( j, i, Color.Lerp( pixel, Color.red, 0.2f ) );
                        }
                        else
                        {
                            frame.Mask[i] = defines.HIST_BG_PIXEL;
                        }
                    }
             //}
                }
            }
        }


        // inpt now is in camera coordinates, it need to be transformed by pose invH to object coordinates
        // inpt is also been properly scaled to math the voxel resolution
        // inpt.w is pf for the point
        private float findPerPixelDT( CameraSpacePoint voxel, float pf, objectPose[] poses, int numObj )
        {
            if (pf > 0)
            {
                float dt = defines.MAX_SDF;
                float partdt = defines.MAX_SDF;
                int idx;
                float[] voxelBlocks;

                for (int i = 0; i < numObj; i++)
                {
                    Vector3 objpt = poses[i].getInvH().MultiplyPoint(new Vector3( voxel.X, voxel.Y, voxel.Z ));
                    idx = pt2IntIdx( objpt );
                    if (idx >= 0)
                    {
                        voxelBlocks = shapes[i].getSDFVoxels();
                        partdt = voxelBlocks[idx];
                        dt = partdt < dt ? partdt : dt;
                    }
                }
                return dt;
            }
            else
            {
                return defines.MAX_SDF;
            }
        }


    }
}


