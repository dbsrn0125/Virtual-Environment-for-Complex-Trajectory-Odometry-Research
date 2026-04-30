using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.LivoxRosDriver2; // 생성된 메시지 네임스페이스
using RosMessageTypes.Std;
using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class LivoxCustomPublisher : MonoBehaviour
{
    [Header("ROS Configuration")]
    public string topicName = "/livox/lidar";
    public string frameId = "livox_frame";

    [Header("LiDAR Settings")]
    [Range(10, 50)]
    public int publishFrequency = 10; // FAST-LIVO2 권장 10Hz
    public int pointsPerFrame = 20000;
    public float maxRange = 50f;
    public float verticalFOV = 360f; // Mid-360 기준
    public float horizontalFOV = 360f;

    private ROSConnection ros;
    private float lastPublishTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<CustomMsgMsg>(topicName);
    }

    void Update()
    {
        if (Time.time - lastPublishTime < 1.0f / publishFrequency) return;

        PublishLivoxData();
        lastPublishTime = Time.time;
    }

    void PublishLivoxData()
    {
        CustomMsgMsg msg = new CustomMsgMsg();

        // 1. 헤더 설정
        msg.header = new HeaderMsg();
        msg.header.frame_id = frameId;

        // 현재 유니티 시간을 ROS 시간으로 변환
        // IMU와 동일하게 Play 버튼을 누른 시점부터의 시간을 사용
        double timeNow = Time.timeAsDouble;
        int sec = (int)Math.Truncate(timeNow);
        uint nanosec = (uint)((timeNow - sec) * 1e9);
        msg.header.stamp = new TimeMsg(sec, nanosec);

        // FAST-LIVO2는 이 timebase를 기준으로 각 포인트의 offset_time을 계산함
        msg.timebase = (ulong)(timeNow * 1e9);
        msg.lidar_id = 1;

        // 2. 가상 포인트 생성 (Raycast 방식)
        List<CustomPointMsg> pointList = new List<CustomPointMsg>();

        for (int i = 0; i < pointsPerFrame; i++)
        {
            // 리보 특유의 스캔 패턴을 흉내내기 위한 무작위 샘플링 (실제 구동시는 스캔 패턴 로직 적용 권장)
            Vector3 rayDir = Quaternion.Euler(UnityEngine.Random.Range(-90, 90), UnityEngine.Random.Range(0, 360), 0) * Vector3.forward;
            RaycastHit hit;

            if (Physics.Raycast(transform.position, transform.TransformDirection(rayDir), out hit, maxRange))
            {
                CustomPointMsg p = new CustomPointMsg();

                // 좌표 변환: Unity (Left-hand, Y-up) -> ROS (Right-hand, Z-up)
                // Unity X (Right) -> ROS -Y
                // Unity Y (Up)    -> ROS Z
                // Unity Z (Fwd)   -> ROS X
                Vector3 localHit = transform.InverseTransformPoint(hit.point);
                p.x = localHit.z;
                p.y = -localHit.x;
                p.z = localHit.y;

                p.reflectivity = (byte)(150); // 가상 반사율
                p.line = (byte)(i % 4);       // 가상 레이저 라인 번호
                p.tag = 0x10;                 // 0x10: 정상 리턴 포인트

                // 포인트별 시간 오프셋 (나노초 단위, 한 프레임 내에서 순차적으로 증가)
                // FAST-LIVO2의 모션 보정에 사용됨
                p.offset_time = (uint)((1.0f / publishFrequency) * 1e9 * (i / (float)pointsPerFrame));

                pointList.Add(p);
            }
        }

        msg.points = pointList.ToArray();
        msg.point_num = (uint)msg.points.Length;

        // 3. ROS 전송
        ros.Publish(topicName, msg);
    }
}