using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace Helpers
{
    public class Cholesky
    {
        private int size;
        private int rank;
        private float[] cholesky;

        public Cholesky( float[] mat, int size)
        {
            this.size = size;
            int size2 = size * size;
            cholesky = new float[size2];

            Array.Copy( mat, cholesky, size2 );

            for (int c = 0; c < size; c++)
            {
                float inv_diag = 1;
                for (int r = c; r < size; r++)
                {
                    float val = cholesky[c + r * size];
                    for (int c2 = 0; c2 < c; c2 ++)
                    {
                        val -= cholesky[c + c2 * size] * cholesky[c2 + r * size];
                    }

                    if ( r == c )
                    {
                        cholesky[c + r * size] = val;
                        if (val == 0) { rank = r; }
                        inv_diag = 1.0f / val;
                    }
                    else
                    {
                        cholesky[r + c * size] = val;
                        cholesky[c + r * size] = val * inv_diag;
                    }
                }
            }

        }

        public void Backsub(float[] result, float[] v)
        {
            float[] y = new float[size];
            for (int i = 0; i < size; i++)
            {
                float val = v[i];
                for (int j = 0; j < i; j++)
                {
                    val -= cholesky[j + i * size] * y[j];
                }
                y[i] = val;
            }
            for (int i = 0; i < size; i ++) { y[i] /= cholesky[i + i * size]; }

            for (int i = size - 1; i >= 0; i--)
            {
                float val = y[i];
                for (int j = i + 1; j < size; j++) { val -= cholesky[i + j * size] * result[j]; }
                result[i] = val;
            }
        }
    }
}
