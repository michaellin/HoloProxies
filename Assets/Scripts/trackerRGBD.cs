using System.Collections;
using System.Collections.Generic;
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
        shapeSDF shape;

        // hold the current frame this is just for convenience so it does 
        // not need to be passed in as function param many times
        frameRGBD frame;

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

        void TrackObjects( frameRGBD frame, shapeSDF shapeUnion, trackerState state, bool updateappearance )
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
                    if (ptcloud_ptr[i].w > 0.5) totalpfpix++;
                }

            }

            energy[0] = totalpfpix > 100 ? e / totalpix : 0.0f;
        }

        /// <summary>
        ///  computePerPixelEnergy finds the energy of a single pixel given a pose and the SDF value of the corresponding voxel
        /// </summary>
        /// <param name="inpt"></param>
        /// <param name="pf"></param>
        /// <param name="pose"></param>
        /// <param name="numObj"></param>
        /// <returns> float enegy value of the input pixel </returns>
        private float computePerPixelEnergy( CameraSpacePoint inpt, float pf, objectPose pose, int numObj )
        {
            // printf("shared energy\n");
            if (pf > 0)
            {
                // TODO michael was working here
                float dt = defines.MAX_SDF, partdt = MAX_SDF;
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
        bool computePerPixelJacobian( out float jacobian, Vector4 pixel, shapeSDF shape, objectPose pose, int numObj )
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
        }

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


