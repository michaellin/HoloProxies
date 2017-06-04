using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using HoloProxies.Engine;
using HoloProxies.Objects;
using HoloProxies.Utils;

/// <summary>
/// The trackingManager implements the main loop that starts off the main engine and runs the process frame function.
/// It also takes user input to start tracking, etc.
/// File from: demo.cpp in Start() and UIEngine.cpp in Update()
/// </summary>
public class trackingManager : MonoBehaviour
{

    private const int numTrackingObj = 1;       // number of objects being tracked
    private FrameManager frame;           // frame manager accesses the kinect data and does all the alignment
    private trackerRGBD tracker;          // tracker that implements the pose estimation algorithm
    private bool needStarTracker = true;  // turn on or off tracking

    //private coreEngine engine;
    //private int[] engineSettings = { histogramNBins, numTrackingObj };
    //private MultiSourceManager msm;

    //public GameObject sourceManager;

    // states for the tracking Manager state machine
    private enum ManagerState : byte
    {
        REINIT_HIST,
        PROCESS_FRAME,
        PROCESS_VIDEO,
        EXIT,
        PROCESS_PAUSED
    }
    private ManagerState currentState = ManagerState.PROCESS_PAUSED;

    private int processedFrameNo = 0;

    // Use this for initialization
    void Start()
    {
        // Get the source manager from which we get out RGBD pixels.
        //msm = sourceManager.GetComponent<MultiSourceManager>();
        //// Start off the main engine
        //engine = new coreEngine( engineSettings, new Vector2i( msm.DepthWidth, msm.DepthHeight ), new Vector2i( msm.ColorWidth, msm.ColorHeight ), DT_VOL_SIZE );

        // Initialize all the important components here
        frame = new FrameManager();
        tracker = new trackerRGBD( numTrackingObj );

        float[] poses = { 0.0f, 0.0f, 0.8f, -Mathf.PI, 0, 0 };
        tracker.trackingState.setHFromParam( poses, 0 );
    }

    // Update is called once per frame
    void Update()
    {

        HandleKeyInput();

        switch (currentState)
        {
            case ManagerState.REINIT_HIST:
                float[] poses = { 0.0f, 0.0f, 0.8f, -Mathf.PI / 2, 0, 0 };
                tracker.trackingState.setHFromParam( poses, 0 );
                updateHistogramFromRendering();
                needStarTracker = true;
                currentState = ManagerState.PROCESS_VIDEO;
                break;

            case ManagerState.PROCESS_FRAME:
                ProcessFrame();
                processedFrameNo++;
                currentState = ManagerState.PROCESS_PAUSED;
                break;

            case ManagerState.PROCESS_VIDEO:
                ProcessFrame();
                processedFrameNo++;
                break;

            //case ManagerState.EXIT:
            //    break;

            case ManagerState.PROCESS_PAUSED:
                break;

        } //end switch

    }

    private void HandleKeyInput()
    {
        if (Input.GetKeyDown( KeyCode.R ))
        {
            Debug.Log( "Re-Initialize histogram" );
            currentState = ManagerState.REINIT_HIST;
        }
        else if (Input.GetKeyDown( KeyCode.N ))
        {
            Debug.Log( "Processing one frame" );
            currentState = ManagerState.PROCESS_FRAME;
        }
        else if (Input.GetKeyDown( KeyCode.B ))
        {
            Debug.Log( "Processing video" );
        }
        else if (Input.GetKeyDown( KeyCode.E ))
        {
            Debug.Log( "Exiting..." );
        }
    }



    private void updateHistogramFromRendering( UChar4Image* rendering, UChar4Image* rgb, LibISR::Objects::ISRHistogram* hist )
    {
        //Vector4u* imgptr = rendering->GetData( MEMORYDEVICE_CPU );
        //Vector4u bpix((uchar)0);
        //for (int i = 0; i < rendering->dataSize; i++)
        //    if (imgptr[i] != bpix) imgptr[i] = Vector4u( 255, 255, 255, 255 );
        //    else imgptr[i] = Vector4u( 100, 100, 100, 100 );

        //hist->buildHistogram( rgb, rendering );
    }
    private void updateHistogramFromRendering()
    {

    }

    #region coreEngine.cpp
    /// <summary>
    /// ProcessFrame is the highest level function loop that call the rest of the tracking
    /// </summary>
    private void ProcessFrame()
    {
        // Start a timer to measure fps
        System.Diagnostics.Stopwatch timer;

        frame.UpdateFrame( tracker.trackingState );

        if (needStarTracker)
        {
            tracker.TrackObjects( frame, tracker.trackingState, true ); // TODO may want to use false for update appearance instead
        }

    }
    #endregion

} // end trackingManager class

