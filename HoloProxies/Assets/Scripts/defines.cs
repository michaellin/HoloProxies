using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace HoloProxies.Utils
{

    public static class defines
    {

        public const int MAX_OBJECT_COUNT = 2;
        public const int HISTOGRAM_NBIN = 16;
        public const int MAX_IMG_PTS = 307200;
        public const int DT_VOL_SIZE = 200;
        public const int DT_VOL_3DSIZE = DT_VOL_SIZE * DT_VOL_SIZE * DT_VOL_SIZE;
        public const int VOL_SCALE = 1000;
        public const int BLOCK_SIZE_SDF = 8;
        public const int BLOCK_SIZE_IMG = 16;
        public const int KINECT_PLAYER_INDEX_SHIFT = 3;
        public const float DTUNE = 0.2f;
        public const float MAX_SDF = 128.0f;
        public const int NUM_OBJ = 1;

		public const short HIST_USELESS_PIXEL = -1;
		public const short HIST_FG_PIXEL = -2;
		public const short HIST_BG_PIXEL = -3;

        public const float TMP_WEIGHT = 1.3f;

        public const int WHITE = 255;
        public const int BLACK = 0;


		public const int BB_MARGIN = 5;
        public const int OUTSIDE_BB = -1;

        public const int DOWNSAMPLE = 4; // remeber to scale K

    }
}
