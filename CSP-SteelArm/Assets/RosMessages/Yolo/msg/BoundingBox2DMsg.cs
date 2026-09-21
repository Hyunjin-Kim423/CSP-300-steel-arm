using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Yolo
{
    public class BoundingBox2DMsg : Message
    {
        public const string k_RosMessageName = "yolo_msgs/BoundingBox2D";
        public override string RosMessageName => k_RosMessageName;

        public Pose2DMsg center = new Pose2DMsg();
        public Vector2Msg size = new Vector2Msg();

        public BoundingBox2DMsg() { }

        public static BoundingBox2DMsg Deserialize(MessageDeserializer deserializer) => new BoundingBox2DMsg(deserializer);

        BoundingBox2DMsg(MessageDeserializer deserializer)
        {
            center = Pose2DMsg.Deserialize(deserializer);
            size = Vector2Msg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(center);
            serializer.Write(size);
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register() => MessageRegistry.Register(k_RosMessageName, Deserialize);
    }
}
