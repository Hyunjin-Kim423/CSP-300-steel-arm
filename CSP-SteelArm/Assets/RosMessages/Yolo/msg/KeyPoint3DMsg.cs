using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Yolo
{
    public class KeyPoint3DMsg : Message
    {
        public const string k_RosMessageName = "yolo_msgs/KeyPoint3D";
        public override string RosMessageName => k_RosMessageName;

        public int id;
        public Geometry.PointMsg point = new Geometry.PointMsg();
        public double score;

        public KeyPoint3DMsg() { }

        public static KeyPoint3DMsg Deserialize(MessageDeserializer deserializer) => new KeyPoint3DMsg(deserializer);

        KeyPoint3DMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out id);
            point = Geometry.PointMsg.Deserialize(deserializer);
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
