using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Yolo
{
    public class MaskMsg : Message
    {
        public const string k_RosMessageName = "yolo_msgs/Mask";
        public override string RosMessageName => k_RosMessageName;

        public int height;
        public int width;
        public Point2DMsg[] data = new Point2DMsg[0];

        public MaskMsg() { }

        public static MaskMsg Deserialize(MessageDeserializer deserializer) => new MaskMsg(deserializer);

        MaskMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out height);
            deserializer.Read(out width);
            deserializer.Read(out data, Point2DMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(height);
            serializer.Write(width);
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
