using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Yolo
{
    public class KeyPoint3DArrayMsg : Message
    {
        public const string k_RosMessageName = "yolo_msgs/KeyPoint3DArray";
        public override string RosMessageName => k_RosMessageName;

        public KeyPoint3DMsg[] data = new KeyPoint3DMsg[0];
        public string frame_id = "";

        public KeyPoint3DArrayMsg() { }

        public static KeyPoint3DArrayMsg Deserialize(MessageDeserializer deserializer) => new KeyPoint3DArrayMsg(deserializer);

        KeyPoint3DArrayMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out data, KeyPoint3DMsg.Deserialize, deserializer.ReadLength());
            deserializer.Read(out frame_id);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(data);
            serializer.Write(data);
            serializer.Write(frame_id);
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register() => MessageRegistry.Register(k_RosMessageName, Deserialize);
    }
}
