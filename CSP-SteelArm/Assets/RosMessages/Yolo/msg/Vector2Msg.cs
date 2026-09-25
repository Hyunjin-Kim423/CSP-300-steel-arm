using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Yolo
{
    public class Vector2Msg : Message
    {
        public const string k_RosMessageName = "yolo_msgs/Vector2";
        public override string RosMessageName => k_RosMessageName;

        public double x;
        public double y;

        public Vector2Msg() { }

        public static Vector2Msg Deserialize(MessageDeserializer deserializer) => new Vector2Msg(deserializer);

        Vector2Msg(MessageDeserializer deserializer)
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
