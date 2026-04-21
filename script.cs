using Microsoft.Kinect;
using System;
using System.Globalization;
using System.IO.Ports;
using System.Net;
using System.Net.Sockets;
using System.Text;
class Program
{
    static void Main(string[] args)
    {
        string targetIp = ""; // <-- Linux IP
        int targetPort = 5005;

        UdpClient udp = new UdpClient();
        IPEndPoint endpoint = new IPEndPoint(IPAddress.Parse(targetIp), targetPort);

        KinectSensor sensor = KinectSensor.GetDefault();

        if (sensor == null)
        {
            Console.WriteLine("Kinect not found");
            return;
        }

        sensor.Open();

        BodyFrameReader bodyReader = sensor.BodyFrameSource.OpenReader();
        Body[] bodies = null;

        Console.WriteLine($"Sending Kinect data to {targetIp}:{targetPort}");

        while (true)
        {
            using (BodyFrame frame = bodyReader.AcquireLatestFrame())
            {
                if (frame == null)
                    continue;

                if (bodies == null)
                    bodies = new Body[frame.BodyCount];

                frame.GetAndRefreshBodyData(bodies);

                foreach (var body in bodies)
                {
                    if (body == null || !body.IsTracked)
                        continue;

                    var joint = body.Joints[JointType.HandRight];

                    if (joint.TrackingState == TrackingState.NotTracked)
                        continue;

                    var pos = joint.Position;

                    var handState = (int)body.HandRightState;
                    var handConfidence = (int)body.HandRightConfidence;
                    string json = string.Format(
                        CultureInfo.InvariantCulture,
                        "{{\"tracked\":true,\"hand_right\":[{0:F3},{1:F3},{2:F3}],\"hand_state\":{3},\"hand_confidence\":{4}}}",
                        pos.X, pos.Y, pos.Z, handState, handConfidence
                    );


                    byte[] data = Encoding.UTF8.GetBytes(json);
                    udp.Send(data, data.Length, endpoint);

                    Console.WriteLine(json);
                    break;
                }
            }
        }
    }
}