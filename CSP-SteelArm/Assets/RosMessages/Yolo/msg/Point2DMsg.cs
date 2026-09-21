using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Yolo
{
    public class Point2DMsg : Message
    {
        public const string k_RosMessageName = "yolo_msgs/Point2D";
        public override string RosMessageName => k_RosMessageName;

        public double x;
        public double y;

        public Point2DMsg() { }

        public static Point2DMsg Deserialize(MessageDeserializer deserializer) => new Point2DMsg(deserializer);

        Point2DMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out x);
            deserializer.Read(out y);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(x);
            serializer.Write(y);
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register() => MessageRegistry.Register(k_RosMessageName, Deserialize);
    }
}
