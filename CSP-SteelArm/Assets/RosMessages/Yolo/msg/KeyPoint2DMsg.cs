using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Yolo
{
    public class KeyPoint2DMsg : Message
    {
        public const string k_RosMessageName = "yolo_msgs/KeyPoint2D";
        public override string RosMessageName => k_RosMessageName;

        public int id;
        public Point2DMsg point = new Point2DMsg();
        public double score;

        public KeyPoint2DMsg() { }

        public static KeyPoint2DMsg Deserialize(MessageDeserializer deserializer) => new KeyPoint2DMsg(deserializer);

        KeyPoint2DMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out id);
            point = Point2DMsg.Deserialize(deserializer);
            deserializer.Read(out score);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(id);
            serializer.Write(point);
            serializer.Write(score);
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register() => MessageRegistry.Register(k_RosMessageName, Deserialize);
    }
}
