using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using HoloProxies.Engine;
using HoloProxies.Utils;

/// <summary>
/// The trackingManager implements the main loop that starts off the main engine and runs the process frame function.
/// It also takes user input to start tracking, etc.
/// File from: demo.cpp in Start() and UIEngine.cpp in Update()
/// </summary>
public class trackingManager : MonoBehaviour
{

    private const int histogramNBins = 16;
    private const int numTrackingObj = 1;
    private const int DT_VOL_SIZE = 200;

    private coreEngine engine;
    private int[] engineSettings = { histogramNBins, numTrackingObj };
    private MultiSourceManager msm;

    public GameObject sourceManager;

	private enum ManagerState:byte 
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
        msm = sourceManager.GetComponent<MultiSourceManager>();
        // Start off the main engine
        engine = new coreEngine( engineSettings, new Vector2i( msm.DepthWidth, msm.DepthHeight ), new Vector2i( msm.ColorWidth, msm.ColorHeight ), DT_VOL_SIZE );

        float[] poses = { 0.0f, 0.0f, 0.8f, -Mathf.PI, 0, 0 };
        engine.trackingState.setHFromParam( poses, 0 );

    }

	// Update is called once per frame
	void Update () {

		HandleKeyInput();

		switch ( currentState ) {
		case ManagerState.REINIT_HIST:
			float[] poses = { 0.0f, 0.0f, 0.8f, -Mathf.PI / 2, 0, 0 }; 
			engine.setHFFromParam (poses, 0);
			//updateHistogramFromRendering() // TODO - seems like only needed for display purposes
			engine.needStarTracker = true;
			currentState = ManagerState.PROCESS_VIDEO;
			break;

			case ManagerState.PROCESS_FRAME:
			engine.ProcessFrame(); 
			currentState = ManagerState.PROCESS_PAUSED;
			break;

			case ManagerState.PROCESS_VIDEO:
			engine.ProcessFrame(); // TODO
			break;

			case ManagerState.EXIT:
			break;

			case ManagerState.PROCESS_PAUSED:
			break;

		} //end switch

	}

	private void HandleKeyInput () {
		if ( Input.GetKeyDown(KeyCode.R) ) {
			Debug.Log("Re-Initialize histogram");
			currentState = ManagerState.REINIT_HIST;
		} else if ( Input.GetKeyDown(KeyCode.N) ) {
			Debug.Log("Processing one frame");
			currentState = ManagerState.PROCESS_FRAME;
		} else if ( Input.GetKeyDown(KeyCode.B) ) {
			Debug.Log("Processing video");
		} else if ( Input.GetKeyDown(KeyCode.E) ) {
			Debug.Log("Exiting...");
		}
	}

	//void inline updateHistogramFromRendering(UChar4Image* rendering, UChar4Image* rgb, LibISR::Objects::ISRHistogram* hist)
	//{
	//	Vector4u* imgptr = rendering->GetData(MEMORYDEVICE_CPU);
	//	Vector4u bpix((uchar)0);
	//	for (int i = 0; i < rendering->dataSize;i++)
	//		if (imgptr[i] != bpix) imgptr[i] = Vector4u(255,255,255,255);
	//		else imgptr[i] = Vector4u(100, 100, 100, 100);
	//	
	//	hist->buildHistogram(rgb, rendering);
	//}
	private void updateHistogramFromRendering( ) {

	}
		
} // end trackingManager class



