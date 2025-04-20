using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Geometry;
using RosMessageTypes.UnityRoboticsDemo;

public class DroneTrajectoryTracker : MonoBehaviour
{
    public Transform drone;
    public GameObject trajectoryParent;
    public string rosTopic = "trajectory_feedback";

    private List<Transform> trajectoryPoints = new List<Transform>();
    private List<float> minDistances = new List<float>();
    private List<Vector3> closestDronePositions = new List<Vector3>();

    private ROSConnection ros;
    private float rosSendTimer = 0f;
    private float rosSendInterval = 1f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<MeasurementPacketMsg>(rosTopic);

        foreach (Transform child in trajectoryParent.transform)
        {
            trajectoryPoints.Add(child);
            minDistances.Add(float.MaxValue);
            closestDronePositions.Add(Vector3.zero);
        }
    }

    void Update()
    {
        for (int i = 0; i < trajectoryPoints.Count; i++)
        {
            float currentDistance = Vector3.Distance(drone.position, trajectoryPoints[i].position);
            if (currentDistance < minDistances[i])
            {
                minDistances[i] = currentDistance;
                closestDronePositions[i] = drone.position;
            }
        }

        rosSendTimer += Time.deltaTime;
        if (rosSendTimer >= rosSendInterval)
        {
            rosSendTimer = 0f;
            SendPackedFeedbackToROS();
        }
    }

    void SendPackedFeedbackToROS()
    {
        MeasurementPointMsg[] pointsArray = new MeasurementPointMsg[trajectoryPoints.Count];

        for (int i = 0; i < trajectoryPoints.Count; i++)
        {
            pointsArray[i] = new MeasurementPointMsg
            {
                index = i,
                min_distance = minDistances[i],
                trajectory_point = new PointMsg(
                    trajectoryPoints[i].position.x,
                    trajectoryPoints[i].position.y,
                    trajectoryPoints[i].position.z
                ),
                closest_position = new PointMsg(
                    closestDronePositions[i].x,
                    closestDronePositions[i].y,
                    closestDronePositions[i].z
                )
            };
        }

        MeasurementPacketMsg packedMsg = new MeasurementPacketMsg { points = pointsArray };
        ros.Publish(rosTopic, packedMsg);
    }
}
