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

    private string[] sdfFiles = { "Data/bin/teacan.bin" };

    private System.Diagnostics.Stopwatch timer;

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
    void Awake()
    {
        // Initialize all the important components here
        frame = new FrameManager();                            // does the frame grabs and frame processing from Kinect
        tracker = new trackerRGBD( numTrackingObj, sdfFiles ); // tracker performs the main tracking algorithm and keeps track of shapes

        float[] poses = { 0.0f, 0.0f, 0.8f, -Mathf.PI, 0, 0 };
        tracker.trackingState.setHFromParam( poses, 0 );       // setting the state of object 1 only

        // Initialize a timer to keep track fo fps
        timer = new System.Diagnostics.Stopwatch();
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
                frame.ReinitHistogramFromRendering( tracker.trackingState );
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
            currentState = ManagerState.PROCESS_VIDEO;

        }
        else if (Input.GetKeyDown( KeyCode.E ))
        {
            Debug.Log( "Exiting..." );
        }
    }


    #region coreEngine.cpp
    /// <summary>
    /// ProcessFrame is the highest level function loop that call the rest of the tracking
    /// </summary>
    private void ProcessFrame()
    {

        timer.Start(); // Start a timer to measure fps
        bool success = frame.UpdateFrame( tracker.trackingState );  //process frame
        timer.Stop();
        Debug.Log( string.Format( "ProcessFrame: {0}", timer.ElapsedMilliseconds ) );

		if (needStarTracker && success)
        {
            tracker.TrackObjects( frame, true ); // TODO may want to use false for update appearance instead
            Debug.Log( tracker.trackingState.energy );
        }
    }
    #endregion

    public Texture2D getColorFrame()
    {
        return frame.ColorTexture;
    }

    public Texture2D getDepthFrame()
    {
        return frame.DepthTexture;
    }

} // end trackingManager class

