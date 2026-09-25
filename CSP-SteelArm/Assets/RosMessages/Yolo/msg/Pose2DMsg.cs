using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Yolo
{
    public class Pose2DMsg : Message
    {
        public const string k_RosMessageName = "yolo_msgs/Pose2D";
        public override string RosMessageName => k_RosMessageName;

        public Point2DMsg position = new Point2DMsg();
        public double theta;

        public Pose2DMsg() { }

        public static Pose2DMsg Deserialize(MessageDeserializer deserializer) => new Pose2DMsg(deserializer);

        Pose2DMsg(MessageDeserializer deserializer)
        {
            position = Point2DMsg.Deserialize(deserializer);
            deserializer.Read(out theta);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(position);
            serializer.Write(theta);
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register() => MessageRegistry.Register(k_RosMessageName, Deserialize);
    }
}
