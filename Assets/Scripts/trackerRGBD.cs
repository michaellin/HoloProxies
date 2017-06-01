using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

using Windows.Kinect;

using HoloProxies.Engine;
using HoloProxies.Utils;
using HoloProxies.Objects;

namespace HoloProxies.Engine
{
    /// <summary>
    /// The trackerRGBD class implements all the algorithm of computing the energy function and minimizing it using
    /// Levenberg-Marquardt.
    /// File from: ISRRGBDTracker.cpp/.h, ISRTracker.h, ISRRGBDTracker_CPU.cpp/.h, ISRRGBDTracker_shared.h
    /// TODO unfinished
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

        // size of the gradient
        int ATb_size; // (6*nObjects)

        // size of the Hessian
        int ATA_size; // (Atb_size^2)

        // Hessian approximated with JTJ
        float[] ATA;

        // gradient
        float[] ATb;

        #region ISRRGBtracker.cpp
        public trackerRGBD( int nObjs )
        {
            nObjects = nObjs;
            ATb_size = nObjs * 6;
            ATA_size = ATb_size * ATb_size;

            ATb = new float[ATb_size];
            ATA = new float[ATA_size];

            trackingState = new trackerState( nObjs );
            tempState = new trackerState( nObjs );
        }

        void computeSingleStep( out float step, float ATA, float ATb, float lambda, int dim )
        {
            step = 0; //TODO
                      //			float *tmpATA = new float[dim*dim];
                      //			for (int i = 0; i < dim*dim; i++) tmpATA[i] = ATA[i];
                      //
                      //			for (int i = 0; i < dim * dim; i += (dim + 1))
                      //			{
                      //				float &ele = tmpATA[i];
                      //				if (!(fabs(ele) < 1e-15f)) ele *= (1.0f + lambda); else ele = lambda*1e-10f;
                      //			}
                      //
                      //			ORUtils::Cholesky cholA(tmpATA, dim);
                      //			cholA.Backsub(step, ATb);
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

        public void TrackObjects( FrameManager frame, shapeSDF[] shapeUnion, trackerState state, bool updateappearance )
        {
            // originally here was:
            //this->shapeUnion = shapeUnion;

            this.frame = frame; // For convenience
            float[] cache = new float[ATb_size];

            float lastenergy = 0;
            float currentenergy = 0;

            bool converged = false;
            float lambda = 10000.0f;

            evaluateEnergy( out lastenergy, state );
            if (lastenergy < 0.1f) { state.energy = 0; return; }

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
                }
            }

            //			//	--------------------------------------------------------------------------
            //
            //			//	 LM
            //
            //			//	--------------------------------------------------------------------------
            //
            //			////	 These are some sensible default parameters for Levenberg Marquardt.
            //			////	 The first three control the convergence criteria, the others might
            //			////	 impact convergence speed.
            //			static const int MAX_STEPS = 100;
            //			static const float MIN_STEP = 0.00005f;
            //			static const float MIN_DECREASE = 0.0001f;
            //			static const float TR_REGION_INCREASE = 0.10f;
            //			static const float TR_REGION_DECREASE = 10.0f;
            //
            //			{// minimalist LM main loop
            //				for (int iter = 0; iter < MAX_STEPS; iter++)
            //				{
            //					computeJacobianAndHessian(ATb_host, ATA_host, tempState);
            //
            //					while (true)
            //					{
            //						computeSingleStep(cacheNabla, ATA_host, ATb_host, lambda, ATb_Size);
            //
            //						// check if step size is very small, if so, converge.
            //						float MAXnorm = 0.0;
            //						for (int i = 0; i<ATb_Size; i++) { float tmp = fabs(cacheNabla[i]); if (tmp>MAXnorm) MAXnorm = tmp; }
            //						if (MAXnorm < MIN_STEP) { converged = true; break; }
            //
            //						tempState->applyIncrementalPoseChangesToInvH(cacheNabla);
            //
            //						evaluateEnergy(&currentenergy, tempState);
            //
            //						if (currentenergy > lastenergy)
            //						{
            //							// check if energy decrease is too small, if so, converge.
            //							if (std::abs(currentenergy - lastenergy) / std::abs(lastenergy+0.01) < MIN_DECREASE) {converged = true;}
            //							lastenergy = currentenergy;
            //							lambda *= TR_REGION_INCREASE;
            //							accpetedState->setFrom(*tempState);
            //							break;
            //						}
            //						else
            //						{
            //							lambda *= TR_REGION_DECREASE;
            //							tempState->setFrom(*accpetedState);
            //						}
            //					}
            //					if (converged) break;
            //
            //				}
            //
            //			}
            //
            //
            //
            //			// after convergence, the w channel of ptcloud is recycled for histogram update
            //			if (lastenergy>=0.5f && updateappearance)
            //			{
            //				lableForegroundPixels(accpetedState);
            //				frame->currentLevel->rgbd->UpdateHostFromDevice();
            //				frame->histogram->updateHistogramFromLabeledRGBD(frame->currentLevel->rgbd, 0.3f, 0.1f);
            //			}
            //
            //			//if (trackerState->numPoses() == 1 && lastenergy > 0.3f && lastenergy < 0.7f)
            //			//{
            //			//	fastReinitialize(lastenergy);
            //			//}
            //
            //			trackerState->setFrom(*accpetedState);
            //			trackerState->energy = lastenergy;
            //			//printf("\tEnergy:%f", lastenergy);
        }
        #endregion


        #region main functions
        /// <summary>
        /// evaluates the total energy of a given frame
        /// </summary>
        /// <param name="energy"></param>
        /// <param name="state"></param>
        private void evaluateEnergy( out float energy, trackerState state )
        {
            int count = frame.Camera3DPoints.Length;

            CameraSpacePoint[] ptcloud_ptr = frame.Camera3DPoints;
            float[] pfArray = frame.PfVec;

            //ISRShape_ptr shapes = this->shapeUnion->getShapeList( false );
            objectPose[] poses = state.getPoseList();
            int objCount = state.numPoses();

            float e = 0, es = 0;
            int totalpix = 0;
            int totalpfpix = 0;

            for (int i = 0; i < count; i++)
            {
                //es = computePerPixelEnergy( frame.Camera3DPoints[i], shapes, poses, objCount );
                es = computePerPixelEnergy( ptcloud_ptr[i], pfArray[i], poses, objCount );
                if (es > 0)
                {
                    e += es; totalpix++;
                    if (pfArray[i] > 0.5) totalpfpix++;
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
        private float computePerPixelEnergy( CameraSpacePoint inpt, float pf, objectPose[] pose, int numObj )
        {
            // TODO Michael left here and also need to implement the hessian and jacobian as helper function
            // printf("shared energy\n");
            if (pf > 0)
            {
                float dt = defines.MAX_SDF, partdt = defines.MAX_SDF;
                int idx;
                float* voxelBlocks;

                for (int i = 0; i < numObj; i++)
                {
                    Vector3f objpt = poses[i].getInvH() * Vector3f( inpt.x, inpt.y, inpt.z );
                    idx = pt2IntIdx( objpt );
                    if (idx >= 0)
                    {
                        voxelBlocks = shapes[i].getSDFVoxel();
                        partdt = voxelBlocks[idx];
                        dt = partdt < dt ? partdt : dt; // now use a hard min to approximate
                                                        // printf("dt: %f \n", dt);
                    }
                }

                if (dt == MAX_SDF) return -1.0f;

                float exp_dt = expf( -dt * DTUNE );
                float deto = exp_dt + 1.0f;
                float sheaviside = 1.0f / deto;
                float sdelta = 4.0f * exp_dt * sheaviside * sheaviside;
                float e = inpt.w * sdelta * TMP_WEIGHT + (1 - inpt.w) * sheaviside * (2 - TMP_WEIGHT);
                return e;
            }
            else return 0.0f;
        }

        #endregion

        #region helper functions
        private void computeJacobianAndHessian( float[] gradient, float[] hession, trackerState tracker )
        {
            int count = frame.Camera3DPoints.Length;
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
                if (computePerPixelJacobian( out jacobian, ptcloud, pfArray, shapes, poses, objCount ))
                {
                    for (int a = 0, counter = 0; a < paramNum; a++)
                    {
                        globalGradient[a] += jacobian[a];
                        for (int b = 0; b <= a; b++, counter++) globalHessian[counter] += jacobian[a] * jacobian[b];
                    }
                }
            }
        }

        /// <summary>
        /// Helper function to compute the jacobian. ptCloud is in camera reference frame and needs to be converted using the pose invH to object coordinates.
        /// pfVec is the probability of each voxel (matches in index)
        /// </summary>
        /// <param name="jacobian"></param>
        /// <param name="ptcloud"></param>
        /// <param name="pfVec"></param>
        /// <param name="shapes"></param>
        /// <param name="poses"></param>
        /// <param name="numObj"></param>
        /// <returns></returns>
        private bool computePerPixelJacobian( out float[] jacobian, CameraSpacePoint ptcloud, float pfVal, shapeSDF[] shapes, objectPose[] poses, int numObj, out float prefix )
        {
            jacobian = new float[numObj * 6];
            prefix = 0;
            if (pfVal < 0) { return false; }
            float dt = defines.MAX_SDF; float partdt = defines.MAX_SDF;
            int idx; int minidx = 0;
            float[] voxelBlocks = new float[defines.DT_VOL_3DSIZE];
            float[] minVoxelBlocks = new float[defines.DT_VOL_3DSIZE];
            Vector3 pt = new Vector3( ptcloud.X, ptcloud.Y, ptcloud.Z );
            Vector3 minpt = new Vector3();
            Vector3 ddt;
            bool minfound = false;
            bool ddtfound = false;

            for (int i = 0; i < numObj; i++)
            {
                Vector3 objpt = poses[i].getInvH() * pt;
                idx = pt2IntIdx( objpt );
                // TODO
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

            prefix = (float) (pfVal * d_delta_dt * defines.TMP_WEIGHT + (1.0f - pfVal) * d_heaviside_dt * (2.0f - defines.TMP_WEIGHT)); // TODO WARNING casting double to float

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
            int x = (int) (pt.x * vol_scale + dt_vol_size / 2.0f - 1.0f + offpt.x);
            int y = (int) (pt.y * vol_scale + dt_vol_size / 2.0f - 1.0f + offpt.y);
            int z = (int) (pt.z * vol_scale + dt_vol_size / 2.0f - 1.0f + offpt.z);

            if (x > 0 && x < dt_vol_size - 1.0f &&
                y > 0 && y < dt_vol_size - 1.0f &&
                z > 0 && z < dt_vol_size - 1.0f)
                return (int) ((z * dt_vol_size + y) * dt_vol_size + x);
            else
                return -1;
        }

        private Vector3 getSDFNormal(Vector3 pt_f, float[] voxelBlock, out bool ddtFound)
        {
            Vector3 ddt = new Vector3();

            float dt1; float dt2;
            int idx;

            idx = pt2IntIdx_offset( pt_f, new Vector3( 1, 0, 0 ) );
            if (idx == -1) { ddtFound = false;  return new Vector3(); }
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


        #region ISRRGBDtracker_CPU.cpp
        //void evaluateEnergy( out float energy, trackerState state )
        //{
        //    energy = 0; // TODO
        //printf( "CPU energy\n" );
        //int count = this->frame->ptCloud->dataSize;
        //Vector4f* ptcloud_ptr = this->frame->ptCloud->GetData( MEMORYDEVICE_CPU );

        //ISRShape_ptr shapes = this->shapeUnion->getShapeList( false );
        //ISRPose_ptr poses = trackerState->getPoseList( false );
        //int objCount = trackerState->numPoses();

        //float e = 0, es = 0;
        //int totalpix = 0;
        //int totalpfpix = 0;

        //for (int i = 0; i < count; i++)
        //{
        //    es = computePerPixelEnergy( ptcloud_ptr[i], shapes, poses, objCount );
        //    if (es > 0)
        //    {
        //        e += es; totalpix++;
        //        if (ptcloud_ptr[i].w > 0.5) totalpfpix++;
        //    }

        //}

        //energy[0] = totalpfpix > 100 ? e / totalpix : 0.0f;
        //}

        //void computeJacobianAndHessian(out float gradient, out float hessian, trackerState state) 
        //{
        //			int count = this->frame->ptCloud->dataSize;
        //			Vector4f* ptcloud_ptr = this->frame->ptCloud->GetData(MEMORYDEVICE_CPU);
        //
        //			ISRShape_ptr shapes = this->shapeUnion->getShapeList(false);
        //			ISRPose_ptr poses = trackerState->getPoseList(false);
        //			int objCount = trackerState->numPoses();
        //
        //			int noPara = trackerState->numPoses() * 6;
        //			int noParaSQ = noPara*noPara;
        //
        //			float *globalGradient = new float[noPara];
        //			float *globalHessian = new float[noParaSQ];
        //			float *jacobian = new float[noPara];
        //
        //			for (int i = 0; i < noPara; i++) globalGradient[i] = 0.0f;
        //			for (int i = 0; i < noParaSQ; i++) globalHessian[i] = 0.0f;
        //
        //			for (int i = 0; i < count; i++)
        //			{
        //				if (computePerPixelJacobian(jacobian, ptcloud_ptr[i], shapes, poses, objCount))
        //				{
        //					for (int a = 0, counter = 0; a < noPara; a++) 	
        //					{
        //						globalGradient[a] += jacobian[a];
        //						for (int b = 0; b <= a; b++, counter++) globalHessian[counter] += jacobian[a] * jacobian[b];
        //					}
        //				}
        //			}
        //
        //			for (int r = 0; r < noPara; ++r) gradient[r] = globalGradient[r];
        //
        //			for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) hessian[r + c * noPara] = globalHessian[counter];
        //			for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) hessian[r + c * noPara] = hessian[c + r*noPara];
        //}

        void lableForegroundPixels( trackerState state )
        {
            //			int count = this->frame->ptCloud->dataSize;
            //			Vector4f* ptcloud_ptr = this->frame->ptCloud->GetData(MEMORYDEVICE_CPU);
            //			Vector4f* rgbd_ptr = this->frame->currentLevel->rgbd->GetData(MEMORYDEVICE_CPU);
            //
            //			ISRShape_ptr shapes = this->shapeUnion->getShapeList(false);
            //			ISRPose_ptr poses = trackerState->getPoseList(false);
            //			int objCount = trackerState->numPoses();
            //
            //			float dt;
            //			int totalpix = 0;
            //
            //			for (int i = 0; i < count; i++)
            //			{
            //				if (ptcloud_ptr[i].w > 0) // in the bounding box and have depth
            //				{
            //					dt = findPerPixelDT(ptcloud_ptr[i], shapes, poses,objCount);
            //					if (fabs(dt) <= 5) { rgbd_ptr[i].w = HIST_FG_PIXEL; }
            //					else { rgbd_ptr[i].w = HIST_BG_PIXEL; }
            //				}
            //				//else
            //				//{
            //				//	rgbd_ptr[i].w = HIST_BG_PIXEL;
            //				//}
            //			}	
        }
        #endregion

        #region ISRRGBDtracker_shared
        // inpt now is in camera coordinates, it need to be transformed by pose invH to object coordinates
        // inpt is also been properly scaled to math the voxel resolution
        // inpt.w is pf for the point
        //float computerPerPixelEnergy( Vector4 pixel, shapeSDF shape, objectPose pose, int numObj )
        //{
        //			// printf("shared energy\n");
        //			if (inpt.w > 0)
        //			{
        //				float dt = MAX_SDF, partdt = MAX_SDF;
        //				int idx;
        //				float *voxelBlocks;
        //
        //				for (int i = 0; i < numObj; i++)
        //				{
        //					Vector3f objpt = poses[i].getInvH()*Vector3f(inpt.x, inpt.y, inpt.z);
        //					idx = pt2IntIdx(objpt);
        //					if (idx >= 0)
        //					{
        //						voxelBlocks = shapes[i].getSDFVoxel();
        //						partdt = voxelBlocks[idx];
        //						dt = partdt < dt ? partdt : dt; // now use a hard min to approximate
        //						// printf("dt: %f \n", dt);
        //					}
        //				}
        //
        //				if (dt == MAX_SDF) return -1.0f;
        //
        //				float exp_dt = expf(-dt * DTUNE);
        //				float deto = exp_dt + 1.0f;
        //				float sheaviside = 1.0f / deto;
        //				float sdelta = 4.0f* exp_dt * sheaviside * sheaviside;
        //				float e = inpt.w * sdelta*TMP_WEIGHT + (1 - inpt.w)*sheaviside*(2-TMP_WEIGHT);
        //				return e;
        //			}
        //			else return 0.0f;
        //}

        // inpt now is in camera coordinates, it need to be transformed by pose invH to object coordinates
        // inpt is also been properly scaled to math the voxel resolution
        // inpt.w is pf for the point
        //bool computePerPixelJacobian( out float jacobian, Vector4 pixel, shapeSDF[] shapes, objectPose pose, int numObj )
        //{
        //			if (inpt.w < 0) return false;
        //
        //			float dt = MAX_SDF, partdt = MAX_SDF;
        //			int idx, minidx;
        //			float *voxelBlocks, *minVoxelBlocks;
        //			Vector3f pt(inpt.x, inpt.y, inpt.z), minpt;
        //			Vector3f ddt;
        //			bool minfound = false, ddtfound = false;
        //
        //			for (int i = 0; i < numObj; i++)
        //			{
        //				Vector3f objpt = poses[i].getInvH()*pt;
        //				idx = pt2IntIdx(objpt);
        //
        //				if (idx >= 0)
        //				{
        //					voxelBlocks = shapes[i].getSDFVoxel();
        //					partdt = voxelBlocks[idx];
        //
        //					if (partdt < dt)
        //					{
        //						minidx = i;
        //						minpt = objpt;
        //						minfound = true;
        //						minVoxelBlocks = voxelBlocks;
        //
        //						dt = partdt;
        //					}
        //				}
        //			}
        //			if (!minfound) return false;
        //
        //			ddt = getSDFNormal(minpt, minVoxelBlocks, ddtfound);
        //			if (!ddtfound) return false;
        //
        //			float exp_dt = expf(-dt * DTUNE);
        //			float deto = exp_dt + 1.0f;
        //			float dbase = exp_dt / (deto * deto);
        //
        //			float d_heaviside_dt = dbase * DTUNE;
        //			float d_delta_dt = 8.0f *DTUNE* expf(-2 * DTUNE*dt) / (deto * deto* deto) - 4 * DTUNE * dbase;
        //
        //			float prefix = inpt.w*d_delta_dt*TMP_WEIGHT + (1 - inpt.w)*d_heaviside_dt*(2-TMP_WEIGHT);
        //
        //			ddt *= prefix;
        //
        //			for (int i = 0; i < numObj * 6; i++) jacobian[i] = 0;
        //			int idxoffset = minidx * 6;
        //
        //			jacobian[idxoffset + 0] = ddt.x;
        //			jacobian[idxoffset + 1] = ddt.y;
        //			jacobian[idxoffset + 2] = ddt.z;
        //			jacobian[idxoffset + 3] = 4.0f * (ddt.z * minpt.y - ddt.y * minpt.z);
        //			jacobian[idxoffset + 4] = 4.0f * (ddt.x * minpt.z - ddt.z * minpt.x);
        //			jacobian[idxoffset + 5] = 4.0f * (ddt.y * minpt.x - ddt.x * minpt.y);
        //
        //			return true;
        //}

        // inpt now is in camera coordinates, it need to be transformed by pose invH to object coordinates
        // inpt is also been properly scaled to math the voxel resolution
        // inpt.w is pf for the point
        bool computePerPixelJacobian( out float jacobian, Vector4 pixel, shapeSDF shape, objectPose pose, int numObj, float prefix )
        {
            //			if (inpt.w < 0) return false;
            //
            //			float dt = MAX_SDF, partdt = MAX_SDF;
            //			int idx, minidx;
            //			float *voxelBlocks, *minVoxelBlocks;
            //			Vector3f pt(inpt.x, inpt.y, inpt.z), minpt;
            //			Vector3f ddt;
            //			bool minfound = false, ddtfound = false;
            //
            //			for (int i = 0; i < numObj; i++)
            //			{
            //				Vector3f objpt = poses[i].getInvH()*pt;
            //				idx = pt2IntIdx(objpt);
            //
            //				if (idx >= 0)
            //				{
            //					voxelBlocks = shapes[i].getSDFVoxel();
            //					partdt = voxelBlocks[idx];
            //
            //					if (partdt < dt)
            //					{
            //						minidx = i;
            //						minpt = objpt;
            //						minfound = true;
            //						minVoxelBlocks = voxelBlocks;
            //
            //						dt = partdt;
            //					}
            //				}
            //			}
            //			if (!minfound) return false;
            //
            //			ddt = getSDFNormal(minpt, minVoxelBlocks, ddtfound);
            //			if (!ddtfound) return false;
            //
            //			float exp_dt = expf(-dt * DTUNE);
            //			float deto = exp_dt + 1.0f;
            //			float dbase = exp_dt / (deto * deto);
            //
            //			float d_heaviside_dt = dbase * DTUNE;
            //			float d_delta_dt = 8.0f *DTUNE* expf(-2 * DTUNE*dt) / (deto * deto* deto) - 4 * DTUNE * dbase;
            //
            //			prefix = inpt.w*d_delta_dt*TMP_WEIGHT + (1 - inpt.w)*d_heaviside_dt*(2 - TMP_WEIGHT);
            //
            //			for (int i = 0; i < numObj * 6; i++) jacobian[i] = 0;
            //			int idxoffset = minidx * 6;
            //
            //			jacobian[idxoffset + 0] = ddt.x;
            //			jacobian[idxoffset + 1] = ddt.y;
            //			jacobian[idxoffset + 2] = ddt.z;
            //			jacobian[idxoffset + 3] = 4.0f * (ddt.z * minpt.y - ddt.y * minpt.z);
            //			jacobian[idxoffset + 4] = 4.0f * (ddt.x * minpt.z - ddt.z * minpt.x);
            //			jacobian[idxoffset + 5] = 4.0f * (ddt.y * minpt.x - ddt.x * minpt.y);
            //
            //			return true;
        }

        // inpt now is in camera coordinates, it need to be transformed by pose invH to object coordinates
        // inpt is also been properly scaled to math the voxel resolution
        // inpt.w is pf for the point
        float findPerPixelDT( Vector4 pixel, shapeSDF shape, objectPose pose, int numObj )
        {
            //			if (inpt.w > 0)
            //			{
            //				float dt = MAX_SDF, partdt = MAX_SDF;
            //				int idx;
            //				float *voxelBlocks;
            //
            //				for (int i = 0; i < numObj; i++)
            //				{
            //					Vector3f objpt = poses[i].getInvH()*Vector3f(inpt.x, inpt.y, inpt.z);
            //					idx = pt2IntIdx(objpt);
            //					if (idx >= 0)
            //					{
            //						voxelBlocks = shapes[i].getSDFVoxel();
            //						partdt = voxelBlocks[idx];
            //						dt = partdt < dt ? partdt : dt; // now use a hard min to approximate
            //					}
            //				}
            //				return dt;
            //			}
            //			else return MAX_SDF;
        }

        #endregion

    }
}


