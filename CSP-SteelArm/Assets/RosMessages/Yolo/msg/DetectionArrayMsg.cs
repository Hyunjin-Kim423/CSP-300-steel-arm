using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Yolo
{
    public class DetectionArrayMsg : Message
    {
        public const string k_RosMessageName = "yolo_msgs/DetectionArray";
        public override string RosMessageName => k_RosMessageName;

        public Std.HeaderMsg header = new Std.HeaderMsg();
        public DetectionMsg[] detections = new DetectionMsg[0];

        public DetectionArrayMsg() { }

        public static DetectionArrayMsg Deserialize(MessageDeserializer deserializer) => new DetectionArrayMsg(deserializer);

        DetectionArrayMsg(MessageDeserializer deserializer)
        {
            header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out detections, DetectionMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(header);
            serializer.WriteLength(detections);
            serializer.Write(detections);
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register() => MessageRegistry.Register(k_RosMessageName, Deserialize);
    }
}
