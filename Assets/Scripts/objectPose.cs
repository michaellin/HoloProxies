using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace HoloProxies.Objects
{
    /// <summary>
    /// The objectPose class encodes the pose of the object
    /// as a 6-DOF transformation from object coordinates
    /// to camera coordinates in the homography matrix H.
    /// File from: ISRPose.h
    /// </summary>
    public class objectPose
    {
        private Matrix4x4 H; // transformation obj->cam
        private Matrix4x4 invH; // inverse transformation cam->obj

        /*** public functions ***/
        public objectPose()
        {
            H = new Matrix4x4();
            invH = new Matrix4x4();
        }

        public void setFromH( Matrix4x4 M )
        {
            H = M;
            invH = H.inverse;
        }

        public void setFromInvH( Matrix4x4 M )
        {
            invH = M;
            H = invH.inverse;
        }

        public void setHFromRT( Vector3 r, Vector3 t )
        {
            H = getProjectionMatrixFromRT( r, t );
            invH = H.inverse;
        }

        public void setHFromParam( float[] param )
        {
            H = getProjectionMatrixFromParam( param );
            invH = H.inverse;
        }

        public void setInvHFromRT( Vector3 r, Vector3 t )
        {
            invH = getProjectionMatrixFromRT( r, t );
            H = invH.inverse;
        }

        public void setInvHFromParam( float[] param )
        {
            invH = getProjectionMatrixFromParam( param );
            H = invH.inverse;
        }

        public Matrix4x4 getH()
        {
            return H;
        }

        public Matrix4x4 getInvH()
        {
            return invH;
        }

        public void applyIncrementalChangeToInvH( Vector3 dr, Vector3 dt )
        {
            Matrix4x4 deltaM = getProjectionMatrixFromRT( dr, dt );
            invH = deltaM * invH;
            H = invH.inverse;
        }

        public void applyIncrementalChangeToInvH( float[] step )
        {
            Matrix4x4 deltaM = getProjectionMatrixFromParam( step );
            invH = deltaM * invH;
            H = invH.inverse;
        }

        public void applyIncrementalRotationToInvHInDegree( Vector3 dr )
        {
            Vector3 r = getMRPfromDegree( dr );
            applyIncrementalChangeToInvH( r, new Vector3() );
        }

        public void applyIncrementalChangeToH( Vector3 dr, Vector3 dt )
        {
            Matrix4x4 deltaM = getProjectionMatrixFromRT( dr, dt );
            H = deltaM * H;
            invH = H.inverse;
        }

        public void applyIncrementalChangeToH( float[] step )
        {
            Matrix4x4 deltaM = getProjectionMatrixFromParam( step );
            H = deltaM * H;
            invH = H.inverse;
        }

        /*** private helper functions ***/
        /// <summary>
        /// Function to convert modified rodriguez parameters to rotation angles.
        /// rotation matrix here is already column major.
        /// </summary>
        private Matrix4x4 getRotationMatrixFromMRP( Vector3 r )
        {
            Matrix4x4 outR = new Matrix4x4();

            float t1 = r.x;
            float t2 = r.y;
            float t3 = r.z;

            float tsq = t1 * t1 + t2 * t2 + t3 * t3;

            float tsum = 1 - tsq;

            outR.m00 = (4 * t1 * t1 - 4 * t2 * t2 - 4 * t3 * t3 + tsum * tsum) / ((1 + tsq) * (1 + tsq));
            outR.m01 = (8 * t1 * t2 + 4 * t3 * tsum) / ((1 + tsq) * (1 + tsq));
            outR.m02 = (8 * t1 * t3 - 4 * t2 * tsum) / ((1 + tsq) * (1 + tsq));
            outR.m10 = (8 * t1 * t2 - 4 * t3 * tsum) / ((1 + tsq) * (1 + tsq));
            outR.m11 = (4 * t2 * t2 - 4 * t1 * t1 - 4 * t3 * t3 + tsum * tsum) / ((1 + tsq) * (1 + tsq));
            outR.m12 = (8 * t2 * t3 + 4 * t1 * tsum) / ((1 + tsq) * (1 + tsq));
            outR.m20 = (8 * t1 * t3 + 4 * t2 * tsum) / ((1 + tsq) * (1 + tsq));
            outR.m21 = (8 * t2 * t3 - 4 * t1 * tsum) / ((1 + tsq) * (1 + tsq));
            outR.m22 = (4 * t3 * t3 - 4 * t2 * t2 - 4 * t1 * t1 + tsum * tsum) / ((1 + tsq) * (1 + tsq));

            return outR;
        }

        /// <summary>
        /// Function to convert rotation and translation into projection matrix.
        /// TODO
        /// Done Alexa
        /// </summary>
        private Matrix4x4 getProjectionMatrixFromRT( Vector3 r, Vector3 t )
        {
            Matrix4x4 outR = getRotationMatrixFromMRP( r );
            Matrix4x4 M = new Matrix4x4();

            M.m00 = outR.m00;
            M.m01 = outR.m01;
            M.m02 = outR.m02;
            M.m03 = 0;

            M.m10 = outR.m10;
            M.m11 = outR.m11;
            M.m12 = outR.m12;
            M.m13 = 0;

            M.m20 = outR.m20;
            M.m21 = outR.m21;
            M.m22 = outR.m22;
            M.m23 = 0;

            M.m30 = t.x;
            M.m31 = t.y;
            M.m32 = t.z;
            M.m33 = 1;

            return M;
        }

        /// <summary>
        /// Function to convert from parameter vector (used by LM) into the
        /// projection matrix calling getProjectionMatrixFromRT.
        /// Get the transformation matrix from pose parameters step = [t' r']
        /// TODO - Done Alexa
        /// </summary>
        private Matrix4x4 getProjectionMatrixFromParam( float[] step )
        {
            Vector3 dt = new Vector3.one * step;
            Vector3 dr = new Vector3.one * step[3];
            return getProjectionMatrixFromRT(dr, dt);
        }

        /// <summary>
        /// Function to convert from degrees of rotation into modified
        /// Rodriguez parameters.
        /// TODO - DONE Alexa
        /// </summary>
        private Vector3 getMRPfromDegree( Vector3 r )
        {
            float rotationX = r.x * DEGTORAD;
            float rotationY = r.y * DEGTORAD;
            float rotationZ = r.z * DEGTORAD;

            float c1 = Mathf.Cos(rotationY / 2);
            float c2 = Mathf.Cos(rotationZ / 2);
            float c3 = Mathf.Cos(rotationX / 2);

            float s1 = Mathf.Sin(rotationY / 2);
            float s2 = Mathf.Sin(rotationZ / 2);
            float s3 = Mathf.Sin(rotationX / 2);

            float c1c2 = c1 * c2;
            float s1s2 = s1 * s2;

            float rotation1 = c1c2*s3 + s1s2*c3;
            float rotation2 = s1*c2*c3 + c1*s2*s3;
            float rotation3 = c1*s2*c3 - s1*c2*s3;
            float rotation4 = c1c2*c3 - s1s2*s3;

            float normal = 1 / Mathf.Sqrt(rotation1 *rotation1 + rotation2 * rotation2 + rotation3 * rotation3 + rotation4 * rotation4);

            float b0 = rotation4 * normal;
            float b1 = rotation1 * normal;
            float b2 = rotation2 * normal;
            float b3 = rotation3 * normal;

            Vector3 theta = new Vector3(b1 / (1 + b0),  b2 / (1 + b0), b3 / (1 + b0))

            return theta;
        }

    }
}
