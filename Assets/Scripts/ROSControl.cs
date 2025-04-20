using System;
using UnityEngine;
using MixedReality.Toolkit;
using MixedReality.Toolkit.Subsystems;
using UnityEngine.XR;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.UnityRoboticsDemo;
using RosMessageTypes.Std;
using MixedReality.Toolkit.SpatialManipulation;
using UnityEngine.SceneManagement;
using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Geometry;


public class ROSControl : MonoBehaviour
{
    private HandsAggregatorSubsystem handAggregatorSubsystem;
    private Transform droneTransform;

    // Reference to the XR Rig camera or head object
    public Transform headTransform;

    public float maxMoveSpeed = 1.0f;
    public float maxTiltAngle = 50.0f;  // Maximum tilt angle for max speed

    private float referenceHeight = 0.0f;
    private bool gestureControlEnabled = false;

    private ObjectManipulator objectManipulator;
    private Rigidbody droneRigidbody;

    public GameObject boundaryCube;
    private bool isBoundaryActive = true;

    ROSConnection ros;

    private List<Vector3> droneTrajectory = new List<Vector3>();
    private Vector3 lastRecordedPosition;
    private bool isTrackingTrajectory = false;

    private LineRenderer lineRenderer;

    void Start()
    {
        // Initialize hand aggregator subsystem
        handAggregatorSubsystem = XRSubsystemHelpers.GetFirstRunningSubsystem<HandsAggregatorSubsystem>();
        droneTransform = this.transform;

        // Ensure Rigidbody is set up for collision control
        droneRigidbody = GetComponent<Rigidbody>();
        if (droneRigidbody == null)
        {
            droneRigidbody = gameObject.AddComponent<Rigidbody>();
        }
        droneRigidbody.useGravity = false;
        droneRigidbody.isKinematic = false;

        objectManipulator = GetComponent<ObjectManipulator>();

        // If not set in the Inspector, attempt to find the main camera in the XR Rig
        if (headTransform == null)
        {
            headTransform = Camera.main?.transform;
        }

        // Initialize ROS connection
        // Create a small red cube for init status
        initStatusCube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        initStatusCube.transform.localScale = Vector3.one * 0.01f;
        initStatusCube.transform.position = headTransform.position + headTransform.forward * 0.5f + Vector3.down * 0.1f;
        initStatusCube.transform.SetParent(headTransform); // Make it follow the user's view
        initStatusCube.GetComponent<Renderer>().material.color = Color.red;

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PosRotMsg>("pos_rot");
        ros.RegisterPublisher<Int32Msg>("control");
        ros.RegisterPublisher<PosRotMsg>("debug");
        ros.RegisterPublisher<PosRotMsg>("hololens_init_success");
        ros.RegisterPublisher<DroneTrajectoryMsg>("drone_trajectory");
        ros.Subscribe<PosRotMsg>("hololens_init_pose", InitOriginFromROS);
        UpdateControlMode();

        optitrackOrigin = new GameObject("OptiTrackOrigin");

        // Only move world objects, not the XR Rig
        string[] objectsToReposition = { "drone", "gas_station", "BoundaryCube", "Fires", "FireBoundaries", "Trajectory"};

        foreach (string name in objectsToReposition)
        {
            GameObject obj = GameObject.Find(name);
            if (obj != null)
            {
                obj.transform.SetParent(optitrackOrigin.transform, true);
            }
        }

        // Create and configure the LineRenderer
        lineRenderer = gameObject.AddComponent<LineRenderer>();
        lineRenderer.startWidth = 0.01f;
        lineRenderer.endWidth = 0.01f;
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.positionCount = 0;
        lineRenderer.useWorldSpace = true;
        lineRenderer.startColor = Color.green;
        lineRenderer.endColor = Color.green;

    }

    private bool originInitialized = false;
    private GameObject optitrackOrigin;
    private GameObject initStatusCube;

    private IEnumerator FadeOutCube(float duration)
    {
        if (initStatusCube == null) yield break;

        Renderer cubeRenderer = initStatusCube.GetComponent<Renderer>();
        Material mat = cubeRenderer.material;
        Color startColor = mat.color;
        float startTime = Time.time;

        while (Time.time < startTime + duration)
        {
            float t = (Time.time - startTime) / duration;
            Color newColor = Color.Lerp(startColor, new Color(startColor.r, startColor.g, startColor.b, 0f), t);
            mat.color = newColor;
            yield return null;
        }

        Destroy(initStatusCube);
    }

    private void InitOriginFromROS(PosRotMsg msg)
    {
        if (originInitialized) return;

        // Get OptiTrack pose
        Vector3 pose_opti = new Vector3(msg.pos_x, msg.pos_y, msg.pos_z);
        Quaternion rot_opti = new Quaternion(msg.rot_x, msg.rot_y, msg.rot_z, msg.rot_w);

        // Get current Unity camera pose
        Transform cam = headTransform != null ? headTransform : Camera.main.transform;
        Vector3 pose_holo = cam.position;
        Quaternion rot_holo = cam.rotation;

        // Transpose origin
        optitrackOrigin.transform.position = pose_holo - pose_opti;
        PosRotMsg offset = new PosRotMsg(
            optitrackOrigin.transform.position.x,
            optitrackOrigin.transform.position.y,
            optitrackOrigin.transform.position.z,
            0,
            0,
            0,
            1
        );
        ros.Publish("hololens_init_success", offset);

        // Rotation TODO
        /*
        PosRotMsg cubePos = new PosRotMsg(
            pose_opti.x,
            pose_opti.y,
            pose_opti.z,
            rot_opti.x,
            rot_opti.y,
            rot_opti.z,
            rot_opti.w
        );
        ros.Publish("debug", cubePos);
        PosRotMsg cubePos2 = new PosRotMsg(
            pose_holo.x,
            pose_holo.y,
            pose_holo.z,
            rot_holo.x,
            rot_holo.y,
            rot_holo.z,
            rot_holo.w
        );
        ros.Publish("debug", cubePos2);
        PosRotMsg cubePos3 = new PosRotMsg(
            optitrackOrigin.transform.position.x,
            optitrackOrigin.transform.position.y,
            optitrackOrigin.transform.position.z,
            0,
            0,
            0,
            1
        );
        ros.Publish("debug", cubePos3);
        */

        //optitrackOrigin.transform.rotation = Quaternion.Inverse(rot_opti);

        originInitialized = true;

        // Turn cube green + fade
        if (initStatusCube != null)
        {
            Renderer cubeRenderer = initStatusCube.GetComponent<Renderer>();
            cubeRenderer.material.color = Color.green;

            Material mat = cubeRenderer.material;
            mat.SetFloat("_Mode", 2);
            mat.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            mat.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            mat.SetInt("_ZWrite", 0);
            mat.DisableKeyword("_ALPHATEST_ON");
            mat.EnableKeyword("_ALPHABLEND_ON");
            mat.DisableKeyword("_ALPHAPREMULTIPLY_ON");
            mat.renderQueue = 3000;

            StartCoroutine(FadeOutCube(2f));
        }
    }

    void Update()
    {
        if (gestureControlEnabled)
        {
            HandleGestureControl();
        }

        // Always send the updated position and orientation
        SendPositionAndOrientation();

        if (isTrackingTrajectory)
        {
            TrackDronePosition();
        }

    }

    private void TrackDronePosition()
    {
        Vector3 currentPos = droneTransform.position;

        if (droneTrajectory.Count == 0 || currentPos != lastRecordedPosition)
        {
            droneTrajectory.Add(currentPos);
            lastRecordedPosition = currentPos;

            // Update line renderer
            lineRenderer.positionCount = droneTrajectory.Count;
            lineRenderer.SetPosition(droneTrajectory.Count - 1, currentPos);
        }
    }

    public void ToggleControlMode()
    {
        gestureControlEnabled = !gestureControlEnabled;
        Debug.Log("Control Mode Toggled: " + (gestureControlEnabled ? "Gesture Control" : "Grab-and-Move"));
        UpdateControlMode();
    }

    private void UpdateControlMode()
    {
        // Toggle ObjectManipulator and physics behavior based on the control mode
        if (gestureControlEnabled)
        {
            // Disable ObjectManipulator and ensure no external forces affect the drone
            if (objectManipulator != null) objectManipulator.enabled = false;
            droneRigidbody.isKinematic = false;
        }
        else
        {
            // Enable ObjectManipulator for free movement
            if (objectManipulator != null) objectManipulator.enabled = true;

            // Disable physics effects like drifting
            droneRigidbody.isKinematic = true;
        }
    }

    private void HandleGestureControl()
    {
        if (handAggregatorSubsystem != null && headTransform != null)
        {
            // Check if the left hand is in view
            XRNode leftHandNode = XRNode.LeftHand;
            if (handAggregatorSubsystem.TryGetJoint(TrackedHandJoint.Palm, leftHandNode, out HandJointPose leftPalmPose))
            {
                // If the left hand is in view, set the reference height to the right hand's current Y position
                XRNode rightHandNode = XRNode.RightHand;
                if (handAggregatorSubsystem.TryGetJoint(TrackedHandJoint.Palm, rightHandNode, out HandJointPose rightPalmPose))
                {
                    referenceHeight = rightPalmPose.Position.y;
                }
                SendPositionAndOrientation(); // Send data every time, so the ROS topic won't stall.
                return; // Disable gesture-based control when the left hand is in view
            }

            // Use the right hand for gesture control only if the left hand is not in view
            if (handAggregatorSubsystem.TryGetJoint(TrackedHandJoint.Palm, XRNode.RightHand, out HandJointPose palmPose))
            {
                // If the left hand is not in view, allow gesture-based control using the right hand
                float heightDifference = palmPose.Position.y - referenceHeight;

                // Clamp height difference to prevent excessive vertical movement
                heightDifference = Mathf.Clamp(heightDifference, -0.5f, 0.5f);

                // Calculate movement direction and tilt-based speed
                Vector3 moveDirection = CalculateMoveDirection(palmPose);

                // Combine height difference with movement direction
                Vector3 combinedMovement = moveDirection * maxMoveSpeed * Time.deltaTime;
                combinedMovement.y += heightDifference * maxMoveSpeed * Time.deltaTime;

                // Move the drone
                droneTransform.Translate(combinedMovement);
            }
            else
            {
                // Reset reference height if the right hand is not detected
                referenceHeight = 0.0f;
            }
        }
    }

    private Vector3 CalculateMoveDirection(HandJointPose palmPose)
    {
        float rollAngle = palmPose.Rotation.eulerAngles.z;
        if (rollAngle > 180) rollAngle -= 360;

        float pitchAngle = palmPose.Rotation.eulerAngles.x;
        if (pitchAngle > 180) pitchAngle -= 360;

        Vector3 moveDirection = new Vector3(-rollAngle, 0, pitchAngle).normalized;
        float tiltMagnitude = Mathf.Sqrt(rollAngle * rollAngle + pitchAngle * pitchAngle);
        float tiltFactor = Mathf.Clamp01(tiltMagnitude / maxTiltAngle);

        Vector3 headsetYaw = headTransform.eulerAngles;
        Quaternion yawRotation = Quaternion.Euler(0, headsetYaw.y, 0);

        return yawRotation * moveDirection * tiltFactor;
    }

    private void SendPositionAndOrientation()
    {
        Vector3 position = droneTransform.position;
        Quaternion orientation = droneTransform.rotation;

        PosRotMsg cubePos = new PosRotMsg(
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        );

        ros.Publish("pos_rot", cubePos);
    }

    private void SendTrajectoryToROS()
    {
        PointMsg[] positions = new PointMsg[droneTrajectory.Count];

        for (int i = 0; i < droneTrajectory.Count; i++)
        {
            Vector3 pos = droneTrajectory[i];
            positions[i] = new PointMsg(pos.x, pos.y, pos.z);
        }

        DroneTrajectoryMsg msg = new DroneTrajectoryMsg
        {
            positions = positions
        };

        ros.Publish("drone_trajectory", msg);
    }

    public void SendTakeoff()
    {
        Int32Msg controlMsg = new Int32Msg(1);
        ros.Publish("control", controlMsg);
        Debug.Log("Takeoff command sent (1)");

        // Start tracking
        isTrackingTrajectory = true;
        droneTrajectory.Clear();
        lastRecordedPosition = Vector3.zero;

        lineRenderer.positionCount = 0;
    }

    public void SendLanding()
    {
        Int32Msg controlMsg = new Int32Msg(0);
        ros.Publish("control", controlMsg);
        Debug.Log("Landing command sent (0)");

        // Stop tracking and send trajectory
        isTrackingTrajectory = false;
        SendTrajectoryToROS();
    }

    public void ToggleBoundary()
    {
        isBoundaryActive = !isBoundaryActive;

        if (boundaryCube != null)
        {
            // Enable or disable the entire GameObject
            boundaryCube.SetActive(isBoundaryActive);
        }

        Debug.Log("Boundary " + (isBoundaryActive ? "Activated" : "Deactivated"));
    }

    public void ResetScene()
    {
        // Reload the current active scene
        SceneManager.LoadScene(SceneManager.GetActiveScene().name);

        Debug.Log("Scene reset to initial state");
    }
}
