using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Yolo
{
    public class KeyPoint2DArrayMsg : Message
    {
        public const string k_RosMessageName = "yolo_msgs/KeyPoint2DArray";
        public override string RosMessageName => k_RosMessageName;

        public KeyPoint2DMsg[] data = new KeyPoint2DMsg[0];

        public KeyPoint2DArrayMsg() { }

        public static KeyPoint2DArrayMsg Deserialize(MessageDeserializer deserializer) => new KeyPoint2DArrayMsg(deserializer);

        KeyPoint2DArrayMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out data, KeyPoint2DMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(data);
            serializer.Write(data);
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register() => MessageRegistry.Register(k_RosMessageName, Deserialize);
    }
}
