using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using HoloProxies.Engine;

/// <summary>
/// The trackingManager implements the main loop that starts off the main engine and runs the process frame function.
/// It also takes user input to start tracking, etc.
/// File from: demo.cpp in Start() and UIEngine.cpp in Update()
/// </summary>
public class trackingManager : MonoBehaviour {

    private coreEngine engine; 

    private ManagerState { REINIT_HIST, PROCESS_FRAME, PROCESS_VIDEO, EXIT, PROCESS_PAUSED };
    private ManagerState currentState = ManagerState.PROCESS_PAUSED;

    private int processedFrameNo = 0;

	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {

		HandleKeyInput();

		switch ( currentState ) {
			case ManagerState.REINIT_HIST:
			float poses[6] = { 0.0f, 0.0f, 0.8f, -PI/2 , 0, 0 };
			engine.setHFFromParam(poses, 0);
			//updateHistogramFromRendering() // TODO
			engine.needStarTracker = true;
			currentState = ManagerState.PROCESS_VIDEO;
			processedFrameNo++;
			break;

			case ManagerState.PROCESS_FRAME:
			engine.ProcessFrame(); 
			currentState = ManagerState.PROCESS_PAUSED;
			break;

			case ManagerState.PROCESS_VIDEO:
			engine.ProcessFrame(); // TODO
			processedFrameNo++;
			break;

			case ManagerState.EXIT:
			break;

			case ManagerState.PROCESS_PAUSED:
			break;

		} //end switch

	}

	private void HandleKeyInput ( void ) {
		if ( Input.GetKeyDown(KeyCode.R) ) {
			Debug.Log("Re-Initialize histogram");
			currentState = ManagerState.REINIT_HIST;
		} else if ( Input.GetKeyDown(KeyCode.N) ) {
			Debug.Log("Processing one frame");
			currentState = ManagerState.PROCESS_FRAME;
		} else if ( Input.GetKeyDown(KeyCode.B) ) {
			Debug.Log("Processing i")
		} else if ( Input.GetKeyDown(KeyCode.E) ) {

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









